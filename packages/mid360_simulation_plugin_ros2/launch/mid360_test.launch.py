from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    share = Path(get_package_share_directory("mid360_simulation_plugin_ros2"))
    world = share / "worlds" / "mid360_test.sdf"
    return LaunchDescription([
        ExecuteProcess(cmd=["gz", "sim", "-r", str(world)], output="screen")
    ])
