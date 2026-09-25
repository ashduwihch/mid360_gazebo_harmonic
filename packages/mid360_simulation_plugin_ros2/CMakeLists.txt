cmake_minimum_required(VERSION 3.16)
project(mid360_simulation_plugin_ros2 VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(rmagine 2.3 REQUIRED COMPONENTS core embree)
find_package(rmagine_gazebo_plugins REQUIRED)
find_package(livox_ros_driver2 QUIET)

add_library(mid360_system SHARED src/mid360_system.cpp)
target_include_directories(mid360_system PRIVATE include)
target_link_libraries(mid360_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  rmagine::core
  rmagine::embree)
ament_target_dependencies(mid360_system
  ament_index_cpp builtin_interfaces rclcpp sensor_msgs std_msgs
  rmagine_gazebo_plugins)

if(livox_ros_driver2_FOUND)
  target_compile_definitions(mid360_system PRIVATE MID360_HAS_LIVOX_CUSTOM_MSG=1)
  ament_target_dependencies(mid360_system livox_ros_driver2)
  message(STATUS "MID-360: livox_ros_driver2 found, CustomMsg output enabled")
else()
  message(STATUS "MID-360: livox_ros_driver2 not found, PointCloud2 output only")
endif()

install(TARGETS mid360_system
  LIBRARY DESTINATION lib)
install(DIRECTORY models scan_mode worlds launch
  DESTINATION share/${PROJECT_NAME})
install(FILES hooks/mid360_paths.dsv
  DESTINATION share/${PROJECT_NAME}/environment)

ament_environment_hooks(hooks/mid360_paths.dsv)
ament_package()
