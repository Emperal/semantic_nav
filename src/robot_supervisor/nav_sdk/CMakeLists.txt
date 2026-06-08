cmake_minimum_required(VERSION 3.8)
project(nav_sdk)

# ========================
# C++标准
# ========================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# ========================
# 查找依赖
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)


# ========================
# include路径
# ========================
include_directories(
  include
)


add_executable(nav_rpc_server
  src/nav_rpc_server.cpp   # 你的这段代码
  src/nav_sdk.cpp    # 你的SDK实现
)

# ========================
# 链接
# ========================
target_link_libraries(nav_rpc_server
  nlohmann_json::nlohmann_json
  Threads::Threads
)

ament_target_dependencies(nav_rpc_server
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  nlohmann_json
  Threads
)

# ========================
# 安装
# ========================
install(TARGETS nav_rpc_server
  DESTINATION lib/${PROJECT_NAME}
)
# ========================
#  生成动态库
# ========================
add_library(nav_sdk SHARED
  src/nav_sdk.cpp
)

# 让库支持 position independent（重要）
set_target_properties(nav_sdk PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

ament_target_dependencies(nav_sdk
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
)

# ========================
# 可执行程序（可选测试）
# ========================
add_executable(nav_sdk_test
  src/main.cpp
)

target_link_libraries(nav_sdk_test
  nav_sdk
)

ament_target_dependencies(nav_sdk_test
  rclcpp
)

# ========================
# 安装
# ========================

# 头文件（给外部用）
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 动态库安装
install(TARGETS nav_sdk
  EXPORT nav_sdk_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 可执行程序
install(TARGETS nav_sdk_test
  DESTINATION lib/${PROJECT_NAME}
)


# ========================
# 导出（给外部工程用）
# ========================
ament_export_targets(nav_sdk_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(nav_sdk)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
)

ament_package()
