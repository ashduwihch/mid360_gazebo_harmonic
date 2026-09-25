#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/common/Console.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sdf/Element.hh>

#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine_gazebo_plugins/gz/map_registry.hpp>

#ifdef MID360_HAS_LIVOX_CUSTOM_MSG
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#endif

#include "mid360_simulation_plugin_ros2/csv_scan_pattern.hpp"

namespace mid360_simulation_plugin_ros2
{
namespace
{
enum class RayBackend
{
  Embree,
  Physics
};

struct RayPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  bool valid{false};
};

using EmbreeResults = rmagine::Bundle<
  rmagine::Points<rmagine::RAM>,
  rmagine::Hits<rmagine::RAM>>;

template<typename T>
T SdfValue(const std::shared_ptr<const sdf::Element> &sdf,
           const std::string &name, const T &fallback)
{
  return sdf->HasElement(name) ? sdf->Get<T>(name) : fallback;
}

template<typename T>
void Write(std::vector<std::uint8_t> &data, std::size_t offset, const T &value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

rmagine::Transform ToRmagineTransform(const gz::math::Pose3d &pose)
{
  rmagine::Transform transform;
  transform.R.x = static_cast<float>(pose.Rot().X());
  transform.R.y = static_cast<float>(pose.Rot().Y());
  transform.R.z = static_cast<float>(pose.Rot().Z());
  transform.R.w = static_cast<float>(pose.Rot().W());
  transform.t.x = static_cast<float>(pose.Pos().X());
  transform.t.y = static_cast<float>(pose.Pos().Y());
  transform.t.z = static_cast<float>(pose.Pos().Z());
  return transform;
}
}  // namespace

class Mid360System final : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate,
                           public gz::sim::ISystemPostUpdate
{
public:
  ~Mid360System() override
  {
    if (this->modelRegisteredForIgnore)
    {
      rmagine_gazebo_plugins::MapRegistry::Instance().RemoveIgnoredModelEntity(
        this->modelEntity);
    }
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm))
    {
      gzerr << "[MID360] Plugin must be attached to a model.\n";
      return;
    }

    const auto linkName = SdfValue<std::string>(sdf, "link_name", "mid360_link");
    this->rayEntity = model.LinkByName(ecm, linkName);
    if (this->rayEntity == gz::sim::kNullEntity)
    {
      gzerr << "[MID360] Link [" << linkName << "] was not found.\n";
      return;
    }

    this->topic = SdfValue<std::string>(sdf, "topic", "/livox/lidar");
    this->frameId = SdfValue<std::string>(sdf, "frame_id", "livox_frame");
    this->imuTopic = SdfValue<std::string>(sdf, "imu_topic", "/livox/imu");
    this->gzImuTopic = SdfValue<std::string>(sdf, "gz_imu_topic", "/mid360/imu_gz");
    this->updateRate = std::max(0.1, SdfValue<double>(sdf, "update_rate", 10.0));
    this->samplesPerScan = std::max(1, SdfValue<int>(sdf, "samples", 20000));
    this->downsample = std::max(1, SdfValue<int>(sdf, "downsample", 1));
    this->minRange = std::max(0.0, SdfValue<double>(sdf, "min_range", 0.1));
    this->maxRange = std::max(this->minRange, SdfValue<double>(sdf, "max_range", 40.0));
    this->pointRate = std::max(1.0, SdfValue<double>(sdf, "point_rate", 200000.0));
    this->publishPointcloudType = SdfValue<int>(sdf, "publish_pointcloud_type", 2);
    this->mapKey = SdfValue<std::string>(sdf, "map_key", "default");

    const auto backendName = SdfValue<std::string>(sdf, "ray_backend", "embree");
    if (backendName == "embree")
    {
      this->rayBackend = RayBackend::Embree;
      this->modelEntity = entity;
      rmagine_gazebo_plugins::MapRegistry::Instance().AddIgnoredModelEntity(entity);
      this->modelRegisteredForIgnore = true;
    }
    else if (backendName == "physics")
    {
      this->rayBackend = RayBackend::Physics;
    }
    else
    {
      gzerr << "[MID360] Unknown ray_backend [" << backendName
            << "]; use [embree] or [physics].\n";
      return;
    }

    std::string csv = SdfValue<std::string>(sdf, "csv_file", "mid360-real-centr.csv");
    if (csv.find('/') == std::string::npos)
    {
      csv = ament_index_cpp::get_package_share_directory(
        "mid360_simulation_plugin_ros2") + "/scan_mode/" + csv;
    }
    try
    {
      this->pattern = LoadScanPattern(csv);
    }
    catch (const std::exception &error)
    {
      gzerr << "[MID360] " << error.what() << '\n';
      return;
    }

    if (this->rayBackend == RayBackend::Physics)
    {
      gz::sim::components::RaycastDataInfo initialData;
      ecm.CreateComponent(this->rayEntity,
        gz::sim::components::RaycastData(std::move(initialData)));
    }

    if (!rclcpp::ok())
    {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
    this->node = std::make_shared<rclcpp::Node>("mid360_gz_sim");
    // Match livox_ros_driver2 and FAST-LIO's reliable CustomMsg/IMU subscribers.
    // Reliable publishers also support best-effort visualization subscribers.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    if (this->publishPointcloudType == 2)
    {
      this->cloudPublisher = this->node->create_publisher<sensor_msgs::msg::PointCloud2>(
        this->topic, qos);
    }
    this->imuPublisher = this->node->create_publisher<sensor_msgs::msg::Imu>(
      this->imuTopic, qos);
#ifdef MID360_HAS_LIVOX_CUSTOM_MSG
    if (this->publishPointcloudType == 3)
    {
      this->customPublisher =
        this->node->create_publisher<livox_ros_driver2::msg::CustomMsg>(this->topic, qos);
    }
#else
    if (this->publishPointcloudType == 3)
    {
      gzwarn << "[MID360] output type 3 requires livox_ros_driver2; "
             << "temporarily publishing PointCloud2 until it is installed.\n";
      this->cloudPublisher = this->node->create_publisher<sensor_msgs::msg::PointCloud2>(
        this->topic, qos);
    }
#endif

    if (!this->gzImuTopic.empty())
    {
      this->gzNode.Subscribe(this->gzImuTopic, &Mid360System::OnImu, this);
    }

    this->configured = true;
    gzmsg << "[MID360] Loaded " << this->pattern.size()
          << " scan directions; " << this->samplesPerScan << " samples at "
          << this->updateRate << " Hz; backend=" << backendName << ".\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused)
      return;

    this->scanRequested = false;

    // RaycastData is consumed by the physics system every simulation step.
    // Clear the previous request immediately so the fallback backend casts
    // one batch per lidar frame, not the same 20k rays at 1 kHz.
    if (this->rayBackend == RayBackend::Physics)
    {
      auto raycast = ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
      if (raycast)
      {
        raycast->Data().rays.clear();
        raycast->Data().results.clear();
      }
    }

    if (info.simTime < this->nextScanTime)
      return;

    if (this->rayBackend == RayBackend::Embree && !this->RefreshEmbreeMap())
    {
      if (!this->mapMissingWarned)
      {
        gzwarn << "[MID360] Waiting for rmagine Embree map [" << this->mapKey
               << "]. Add librmagine_embree_map_system.so to the world.\n";
        this->mapMissingWarned = true;
      }
      return;
    }

    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / this->updateRate));
    this->nextScanTime = info.simTime + period;
    this->scanStamp = info.simTime;
    this->activeSamples.clear();
    this->activeSamples.reserve(
      static_cast<std::size_t>((this->samplesPerScan + this->downsample - 1) /
      this->downsample));

    for (int i = 0; i < this->samplesPerScan; i += this->downsample)
    {
      const auto patternIndex = (this->startIndex + static_cast<std::size_t>(i)) %
        this->pattern.size();
      const auto &sample = this->pattern[patternIndex];
      this->activeSamples.push_back(sample);
    }
    this->startIndex = (this->startIndex + static_cast<std::size_t>(this->samplesPerScan)) %
      this->pattern.size();

    if (this->rayBackend == RayBackend::Embree)
    {
      this->RunEmbree(ecm);
      return;
    }

    auto raycast = ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
    if (!raycast)
      return;
    raycast->Data().rays.clear();
    raycast->Data().results.clear();
    raycast->Data().rays.reserve(this->activeSamples.size());
    for (const auto &sample : this->activeSamples)
    {
      raycast->Data().rays.push_back({
        sample.direction * this->minRange,
        sample.direction * this->maxRange});
    }
    this->scanRequested = true;
  }

  void PostUpdate(const gz::sim::UpdateInfo &,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || this->rayBackend != RayBackend::Physics ||
        !this->scanRequested)
      return;
    const auto raycast = ecm.Component<gz::sim::components::RaycastData>(this->rayEntity);
    if (!raycast || raycast->Data().results.size() != this->activeSamples.size())
      return;
    this->physicsPoints.resize(raycast->Data().results.size());
    for (std::size_t i = 0; i < raycast->Data().results.size(); ++i)
    {
      const auto &source = raycast->Data().results[i];
      auto &point = this->physicsPoints[i];
      point.valid = std::isfinite(source.fraction) && source.fraction > 0.0 &&
        source.fraction < 1.0;
      point.x = static_cast<float>(source.point.X());
      point.y = static_cast<float>(source.point.Y());
      point.z = static_cast<float>(source.point.Z());
    }
    this->PublishCloud(this->physicsPoints);
    this->scanRequested = false;
  }

private:
  bool RefreshEmbreeMap()
  {
    auto &registry = rmagine_gazebo_plugins::MapRegistry::Instance();
    const auto map = registry.GetEmbreeMap(this->mapKey);
    if (!map)
      return false;

    if (map != this->embreeMap)
    {
      this->embreeMap = map;
      this->embreeMapMutex = registry.GetMapMutex(this->mapKey);
      this->embreeSimulator =
        std::make_unique<rmagine::O1DnSimulatorEmbree>(this->embreeMap);
      this->embreeSimulator->setTsb(rmagine::Transform::Identity());
    }
    this->mapMissingWarned = false;
    return true;
  }

  void RunEmbree(const gz::sim::EntityComponentManager &ecm)
  {
    const auto pointCount = this->activeSamples.size();
    this->embreeModel.width = static_cast<std::uint32_t>(pointCount);
    this->embreeModel.height = 1;
    this->embreeModel.range.min = static_cast<float>(this->minRange);
    this->embreeModel.range.max = static_cast<float>(this->maxRange);
    this->embreeModel.orig = {0.0F, 0.0F, 0.0F};
    this->embreeModel.dirs.resize(pointCount);
    for (std::size_t i = 0; i < pointCount; ++i)
    {
      const auto &direction = this->activeSamples[i].direction;
      this->embreeModel.dirs[i] = {
        static_cast<float>(direction.X()),
        static_cast<float>(direction.Y()),
        static_cast<float>(direction.Z())};
    }

    this->embreeSimulator->setModel(this->embreeModel);
    if (this->embreeResults.points.size() != pointCount)
    {
      rmagine::resize_memory_bundle<rmagine::RAM>(
        this->embreeResults, pointCount, 1, 1);
    }

    const auto sensorPose = gz::sim::worldPose(this->rayEntity, ecm);
    const auto sensorToWorld = ToRmagineTransform(sensorPose);
    const auto wallStart = std::chrono::steady_clock::now();
    {
      std::shared_lock<std::shared_mutex> lock;
      if (this->embreeMapMutex)
      {
        lock = std::shared_lock<std::shared_mutex>(*this->embreeMapMutex);
      }
      this->embreeSimulator->simulate(sensorToWorld, this->embreeResults);
    }
    const auto elapsed = std::chrono::steady_clock::now() - wallStart;
    this->raycastWallTime +=
      std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
    ++this->raycastCount;

    this->embreePoints.resize(pointCount);
    for (std::size_t i = 0; i < pointCount; ++i)
    {
      const auto &source = this->embreeResults.points[i];
      auto &point = this->embreePoints[i];
      point.valid = this->embreeResults.hits[i] != 0 &&
        std::isfinite(source.x) && std::isfinite(source.y) && std::isfinite(source.z);
      point.x = source.x;
      point.y = source.y;
      point.z = source.z;
    }
    this->PublishCloud(this->embreePoints);

    if (this->raycastCount % 100 == 0)
    {
      const double averageMs = static_cast<double>(this->raycastWallTime.count()) /
        static_cast<double>(this->raycastCount) / 1e6;
      gzmsg << "[MID360] Embree average raycast: " << averageMs << " ms for "
            << pointCount << " rays.\n";
    }
  }

  builtin_interfaces::msg::Time RosTime(std::chrono::steady_clock::duration simTime) const
  {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(simTime).count();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  }

  void PublishCloud(const std::vector<RayPoint> &results)
  {
    constexpr std::uint32_t pointStep = 26;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = this->RosTime(this->scanStamp);
    cloud.header.frame_id = this->frameId;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = pointStep;
    const auto field = [](const std::string &name, std::uint32_t offset,
                          std::uint8_t datatype)
    {
      sensor_msgs::msg::PointField result;
      result.name = name;
      result.offset = offset;
      result.datatype = datatype;
      result.count = 1;
      return result;
    };
    cloud.fields = {
      field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
      field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
      field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
      field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
      field("tag", 16, sensor_msgs::msg::PointField::UINT8),
      field("line", 17, sensor_msgs::msg::PointField::UINT8),
      field("timestamp", 18, sensor_msgs::msg::PointField::FLOAT64)};
    cloud.data.reserve(results.size() * pointStep);

#ifdef MID360_HAS_LIVOX_CUSTOM_MSG
    livox_ros_driver2::msg::CustomMsg custom;
    custom.header = cloud.header;
    const auto baseNs = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(this->scanStamp).count());
    custom.timebase = baseNs;
    custom.lidar_id = 0;
    custom.points.reserve(results.size());
#endif

    for (std::size_t i = 0; i < results.size(); ++i)
    {
      const auto &result = results[i];
      if (!result.valid)
        continue;

      const float x = result.x;
      const float y = result.y;
      const float z = result.z;
      const float intensity = 100.0F;
      const std::uint8_t tag = 0x10;
      const std::uint8_t line = this->activeSamples[i].line;
      const auto offsetNs = static_cast<std::uint32_t>(
        std::llround(static_cast<double>(i * this->downsample) * 1e9 / this->pointRate));
      const double timestamp = static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(this->scanStamp).count() +
        offsetNs);

      const auto oldSize = cloud.data.size();
      cloud.data.resize(oldSize + pointStep);
      Write(cloud.data, oldSize + 0, x);
      Write(cloud.data, oldSize + 4, y);
      Write(cloud.data, oldSize + 8, z);
      Write(cloud.data, oldSize + 12, intensity);
      Write(cloud.data, oldSize + 16, tag);
      Write(cloud.data, oldSize + 17, line);
      Write(cloud.data, oldSize + 18, timestamp);

#ifdef MID360_HAS_LIVOX_CUSTOM_MSG
      if (this->customPublisher)
      {
        livox_ros_driver2::msg::CustomPoint point;
        point.offset_time = offsetNs;
        point.x = x;
        point.y = y;
        point.z = z;
        point.reflectivity = 100;
        point.tag = tag;
        point.line = line;
        custom.points.push_back(point);
      }
#endif
    }

    cloud.width = static_cast<std::uint32_t>(cloud.data.size() / pointStep);
    cloud.row_step = cloud.width * cloud.point_step;
    if (this->cloudPublisher)
      this->cloudPublisher->publish(cloud);
#ifdef MID360_HAS_LIVOX_CUSTOM_MSG
    if (this->customPublisher)
    {
      custom.point_num = static_cast<std::uint32_t>(custom.points.size());
      this->customPublisher->publish(custom);
    }
#endif
  }

  void OnImu(const gz::msgs::IMU &input)
  {
    if (!this->imuPublisher)
      return;
    sensor_msgs::msg::Imu output;
    output.header.frame_id = this->frameId;
    if (input.has_header() && input.header().has_stamp())
    {
      output.header.stamp.sec = input.header().stamp().sec();
      output.header.stamp.nanosec = input.header().stamp().nsec();
    }
    output.orientation.w = input.orientation().w();
    output.orientation.x = input.orientation().x();
    output.orientation.y = input.orientation().y();
    output.orientation.z = input.orientation().z();
    output.angular_velocity.x = input.angular_velocity().x();
    output.angular_velocity.y = input.angular_velocity().y();
    output.angular_velocity.z = input.angular_velocity().z();
    output.linear_acceleration.x = input.linear_acceleration().x();
    output.linear_acceleration.y = input.linear_acceleration().y();
    output.linear_acceleration.z = input.linear_acceleration().z();
    this->imuPublisher->publish(output);
  }

  bool configured{false};
  bool scanRequested{false};
  bool mapMissingWarned{false};
  bool modelRegisteredForIgnore{false};
  int publishPointcloudType{2};
  RayBackend rayBackend{RayBackend::Embree};
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity rayEntity{gz::sim::kNullEntity};
  std::vector<ScanSample> pattern;
  std::vector<ScanSample> activeSamples;
  std::size_t startIndex{0};
  int samplesPerScan{20000};
  int downsample{1};
  double minRange{0.1};
  double maxRange{40.0};
  double updateRate{10.0};
  double pointRate{200000.0};
  std::string topic;
  std::string frameId;
  std::string imuTopic;
  std::string gzImuTopic;
  std::string mapKey{"default"};
  std::chrono::steady_clock::duration nextScanTime{0};
  std::chrono::steady_clock::duration scanStamp{0};
  std::vector<RayPoint> physicsPoints;
  std::vector<RayPoint> embreePoints;
  rmagine::EmbreeMapPtr embreeMap;
  std::shared_ptr<std::shared_mutex> embreeMapMutex;
  std::unique_ptr<rmagine::O1DnSimulatorEmbree> embreeSimulator;
  rmagine::O1DnModel embreeModel;
  EmbreeResults embreeResults;
  std::chrono::nanoseconds raycastWallTime{0};
  std::uint64_t raycastCount{0};
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
#ifdef MID360_HAS_LIVOX_CUSTOM_MSG
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr customPublisher;
#endif
  gz::transport::Node gzNode;
};
}  // namespace mid360_simulation_plugin_ros2

GZ_ADD_PLUGIN(
  mid360_simulation_plugin_ros2::Mid360System,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  mid360_simulation_plugin_ros2::Mid360System,
  "mid360_simulation_plugin_ros2::Mid360System")
