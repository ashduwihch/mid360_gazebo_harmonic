# mid360_simulation_plugin_ros2

This is the MID-360 runtime component of the
`mid360_gazebo_harmonic` repository. Build and usage instructions are kept in
the repository root [`README.md`](../../../README.md).

Runtime features retained here:

- recorded non-repeating MID-360 scan order;
- 200,000 points/s at 10 Hz;
- Livox line IDs, per-point offsets, tags and reflectivity;
- `livox_ros_driver2/msg/CustomMsg` and `sensor_msgs/msg/PointCloud2` output;
- simulated IMU output;
- Embree collision-geometry backend and Gazebo physics fallback;
- automatic exclusion of the sensor's owning vehicle.

