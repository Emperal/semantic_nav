cmake_minimum_required(VERSION 3.8)
project(pc2map_matcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)


find_package(PCL REQUIRED COMPONENTS
  common
  io
  registration
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

# 1) include path
include_directories(/usr/include/modbus)

# 2) find libmodbus.so
find_library(MODBUS_LIB modbus
  PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
)

if(NOT MODBUS_LIB)
  message(FATAL_ERROR "libmodbus not found! Please install: sudo apt install libmodbus-dev")
endif()

# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)


add_executable(publish_initial_pose
  src/scan_map_matcher.cpp
)

target_link_libraries(publish_initial_pose ${MODBUS_LIB})

ament_target_dependencies(publish_initial_pose
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_ros
  PCL
  OpenCV
  pcl_conversions
  yaml-cpp
  rclcpp_action
  nav2_msgs
)

add_executable(pcd2map_alignment
  src/pcd2map_alignment.cpp
  src/utilities.cpp
)

ament_target_dependencies(pcd2map_alignment
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_ros
  PCL
  OpenCV
  pcl_conversions
  yaml-cpp
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pcd2map_alignment
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

install(TARGETS
  pcd2map_alignment
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  publish_initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY map DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
