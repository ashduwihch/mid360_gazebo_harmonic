cmake_minimum_required(VERSION 3.16)
project(rmagine_gazebo_plugins VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(rmagine 2.3 REQUIRED COMPONENTS core embree)

add_library(rmagine_embree_map_system SHARED
  src/gz/rmagine_embree_map_system.cpp)
target_include_directories(rmagine_embree_map_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rmagine_embree_map_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  gz-common5::graphics
  rmagine::core
  rmagine::embree)
target_compile_definitions(rmagine_embree_map_system PRIVATE
  RMAGINE_GZ_IGNITION_ERA=0)

install(TARGETS rmagine_embree_map_system
  LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rmagine)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
