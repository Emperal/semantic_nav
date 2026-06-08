cmake_minimum_required(VERSION 3.8)
project(semantic_mapping_pkg)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  semantic_static_map_node
  src/semantic_static_map_node.cpp
)

ament_target_dependencies(
  semantic_static_map_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticObject.msg"
  "msg/SemanticObjectArray.msg"
  "msg/RecognizedObject.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

target_link_libraries(
  semantic_static_map_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)

install(
  TARGETS semantic_static_map_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()