import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


# ======================== 用户配置区 ========================
# PX4-Autopilot 根目录。PX4 不在主目录时，在这里修改。
PX4_DIR = Path.home() / "PX4-Autopilot"

# 默认仿真世界。可改为自己 world 文件的绝对路径。
DEFAULT_WORLD = PX4_DIR / "Tools" / "simulation" / "gz" / "worlds" / "default.sdf"

# PX4 模型名称和自启动配置。4019 是 PX4 自带的 X500 云台配置。
PX4_MODEL = "gz_x500_gimbal_mid360"
PX4_AUTOSTART_ID = "4019"

# 飞机默认出生坐标，单位为米。
DEFAULT_X = "0"
DEFAULT_Y = "0"
DEFAULT_Z = "0.3"

# True：显示 Gazebo 界面；False：仅运行无界面仿真。
DEFAULT_GUI = True

# NVIDIA 独立显卡用户保持 True；其他显卡可改为 False。
USE_NVIDIA_GPU = True
# ============================================================


def _prepend_unique(current, entries):
    paths = []
    for entry in [*entries, *current.split(":")]:
        if entry and entry not in paths:
            paths.append(entry)
    return ":".join(paths)


def _prepare_server_config(px4_dir, world):
    world_text = world.read_text(encoding="utf-8", errors="ignore")
    if "RmagineEmbreeMapSystem" in world_text:
        return None

    source = px4_dir / "Tools" / "simulation" / "gz" / "server.config"
    if not source.exists():
        raise RuntimeError(f"找不到 PX4 Gazebo 配置: {source}")

    plugin = """
    <plugin entity_name="*" entity_type="world"
            filename="librmagine_embree_map_system.so"
            name="rmagine_gazebo_plugins::RmagineEmbreeMapSystem">
      <update>
        <rate_limit>30</rate_limit>
        <delta_trans>0.001</delta_trans>
        <delta_rot>0.001</delta_rot>
      </update>
    </plugin>"""
    config = source.read_text(encoding="utf-8").replace(
        "  </plugins>", f"{plugin}\n  </plugins>"
    )
    target = Path("/tmp") / f"mid360_gz_server_{os.getuid()}.config"
    target.write_text(config, encoding="utf-8")
    return target


def _start(context):
    px4_dir = PX4_DIR
    world = Path(LaunchConfiguration("world").perform(context)).expanduser()
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in {
        "1", "true", "yes", "on"
    }

    share = Path(get_package_share_directory("mid360_simulation_plugin_ros2"))
    model_root = share / "models"
    px4_build = px4_dir / "build" / "px4_sitl_default"
    px4_binary = px4_build / "bin" / "px4"
    px4_rootfs = px4_build / "rootfs"
    px4_plugins = px4_build / "src" / "modules" / "simulation" / "gz_plugins"

    required = {
        "world 文件": world,
        "PX4 SITL": px4_binary,
        "PX4 rootfs": px4_rootfs,
        "X500 MID-360 模型": model_root / "x500_gimbal_mid360" / "model.sdf",
    }
    for description, path in required.items():
        if not path.exists():
            raise RuntimeError(f"找不到{description}: {path}")

    sim_env = os.environ.copy()
    sim_env["GZ_SIM_RESOURCE_PATH"] = _prepend_unique(
        sim_env.get("GZ_SIM_RESOURCE_PATH", ""),
        [
            str(px4_dir / "custom_gazebo" / "models"),
            str(px4_dir / "Tools" / "simulation" / "gz" / "models"),
            str(model_root),
        ],
    )
    sim_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = _prepend_unique(
        sim_env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        [str(px4_plugins)],
    )
    server_config = _prepare_server_config(px4_dir, world)
    if server_config is not None:
        sim_env["GZ_SIM_SERVER_CONFIG_PATH"] = str(server_config)

    gz_env = sim_env.copy()
    if USE_NVIDIA_GPU:
        gz_env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        gz_env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    gz_cmd = ["gz", "sim", "-r"]
    if not gui:
        gz_cmd.append("-s")
    gz_cmd.append(str(world))

    px4_env = sim_env.copy()
    px4_env.update({
        "PX4_GZ_STANDALONE": "1",
        "PX4_SYS_AUTOSTART": PX4_AUTOSTART_ID,
        "PX4_SIM_MODEL": PX4_MODEL,
        "PX4_GZ_MODELS": str(model_root),
        "PX4_GZ_WORLDS": str(world.parent),
        "PX4_GZ_MODEL_POSE": f"{x},{y},{z}",
        "GZ_IP": "127.0.0.1",
    })

    return [
        ExecuteProcess(cmd=gz_cmd, env=gz_env, output="screen"),
        ExecuteProcess(
            cmd=[str(px4_binary)],
            cwd=str(px4_rootfs),
            env=px4_env,
            output="screen",
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=str(DEFAULT_WORLD)),
        DeclareLaunchArgument("x", default_value=DEFAULT_X),
        DeclareLaunchArgument("y", default_value=DEFAULT_Y),
        DeclareLaunchArgument("z", default_value=DEFAULT_Z),
        DeclareLaunchArgument("gui", default_value=str(DEFAULT_GUI).lower()),
        OpaqueFunction(function=_start),
    ])
