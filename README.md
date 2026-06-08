# semantic_nav

`semantic_nav` 是一个面向移动机器人语义导航的 ROS 2 工作空间。项目把底盘驱动、LiDAR/相机数据处理、SLAM/定位、Nav2 导航、语义地图、目标物体检索和 RPC 控制接口整合在一起，用于实现“构建地图 → 生成/加载语义地图 → 按语义目标导航”的完整流程。

> 当前仓库包含多套与具体设备、主机路径和 Conda 环境绑定的脚本。首次部署时请先根据实际机器人、传感器、地图路径和模型路径调整配置。

## 功能概览

- **底盘与里程计**：集成 DT-01-Pro AGV 底盘 ROS 2 驱动、自定义消息和轮速里程计发布。
- **传感器接入与标定**：支持 Livox MID360、Orbbec RGB-D 相机、LiDAR-Camera 外参标定、深度图到彩色图对齐。
- **SLAM 与定位**：包含 LIO-SAM MID360、点云转 LaserScan、slam_toolbox、NDT/点云地图匹配等模块。
- **Nav2 导航**：提供 2D/3D 导航 launch、Nav2 参数、地图保存和 RViz 配置。
- **语义感知与语义地图**：包含 YOLOE 分割、Qwen/SAM 视觉理解、pySLAM 数据导出、语义静态地图节点和语义对象消息。
- **RPC/SDK 控制**：提供 BSON-RPC 导航服务、C++ `nav_sdk`、语义目标查询与导航 RPC 桥接。

## 目录结构

```text
semantic_nav/
├── src/
│   ├── bringup/                         # 一键启动、标定、SLAM、导航 launch 与 RViz/权重资源
│   ├── chassis_semantic/                # 语义感知、语义地图、Orbbec 导出、Qwen 导航、深度彩色对齐
│   ├── chassis_slam_and_nav/            # SLAM、定位、Nav2、点云处理、自定义 costmap 插件
│   ├── robot_description/               # 机器人 URDF、网格模型和可视化配置
│   ├── robot_driver/                    # 底盘驱动与底盘消息
│   └── robot_supervisor/                # 导航 RPC 服务、BSON-RPC、C++ 导航 SDK
├── calibration.sh                       # 启动 LiDAR-Camera 标定流程
├── mapping.sh                           # 启动建图流程
├── nav_final.sh                         # 启动最终导航流程
├── semantic_nav_rpc_server.sh           # 启动语义对象导航 RPC 服务
├── nav_rpc_server.sh                    # 启动通用导航 RPC 服务
├── save_map.sh                          # 保存 Nav2 地图到 chassis_nav/maps
├── yoloe_seg.sh                         # 启动 YOLOE 语义分割/目标导航客户端
├── pyslam.sh                            # 启动 pySLAM
└── tum_dataset_generator.sh             # 导出 Orbbec RGB-D TUM 数据集
```

## 主要 ROS 2 包

| 包 | 位置 | 作用 |
| --- | --- | --- |
| `bringup` | `src/bringup` | 汇总启动文件，串联底盘、LiDAR、SLAM、Nav2、定位和语义导航。 |
| `align_depth2color` | `src/chassis_semantic/align_depth2color` | 基于 TF 将深度图对齐到彩色相机坐标。 |
| `lidar_camera_manual_calib` | `src/chassis_semantic/lidar_camera_manual_calib` | 手动 PnP LiDAR-Camera 外参标定工具。 |
| `orbbec_pyslam_exporter` | `src/chassis_semantic/orbbec_pyslam_exporter` | 将 Orbbec RGB-D 流导出为 TUM RGB-D 数据集格式，供 pySLAM 使用。 |
| `semantic_mapping_pkg` | `src/chassis_semantic/semantic_mapping_pkg` | 定义语义对象消息并维护语义静态地图节点。 |
| `semantic_object_nav_rpc` | `src/chassis_semantic/semantic_object_nav_rpc` | 从 pySLAM 语义 NPZ/PLY 地图中检索目标，并通过 RPC 调用 Nav2 导航。 |
| `qwen_nav_pkg` | `src/chassis_semantic/qwen_nav_pkg` | 接入 Qwen/SAM 类视觉语义理解节点。 |
| `chassis_nav` | `src/chassis_slam_and_nav/chassis_nav` | Nav2 参数、地图、RViz 和导航 launch。 |
| `ekf_filter` | `src/chassis_slam_and_nav/ekf_filter` | 融合/发布轮速里程计和 TF。 |
| `lio_sam_mid360` | `src/chassis_slam_and_nav/lio_sam_mid360` | 面向 Livox MID360 的 LIO-SAM 建图模块。 |
| `pc_to_scan_and_slam` | `src/chassis_slam_and_nav/pc_to_scan_and_slam` | 点云转 LaserScan、过滤和 slam_toolbox 建图。 |
| `pc2map_matcher` / `ndt_localization` | `src/chassis_slam_and_nav` | 点云地图匹配和 NDT 定位。 |
| `dt_ros2` / `robot_ros2_msgs` | `src/robot_driver/chassis_driver` | DT-01-Pro 底盘驱动和底盘消息定义。 |
| `bson_rpc` | `src/robot_supervisor/bson_rpc` | Python BSON-RPC Nav2 服务端。 |
| `nav_sdk` | `src/robot_supervisor/nav_sdk` | C++ Nav2 RPC 服务端和导航 SDK。 |

## 环境要求

### 基础环境

- Ubuntu + ROS 2（脚本中默认使用 `/opt/ros/jazzy/setup.bash`，建议 ROS 2 Jazzy 环境）。
- `colcon`、`rosdep`、`ament_cmake`、`ament_python`。
- Nav2、slam_toolbox、tf2、RViz2、PCL、OpenCV、Eigen、GTSAM、nlohmann_json 等 ROS/C++ 依赖。
- Livox MID360 ROS 2 驱动（`livox_ros_driver2`）。
- Orbbec 相机驱动及相关 ROS 2 话题。
- Python/Conda 环境：
  - `itemid`：用于 YOLOE 语义分割脚本。
  - `pyslam`：用于 pySLAM。
  - 远端 Qwen 模型环境（示例脚本中为 `model_qwen_310`）。

### Python 依赖

`semantic_object_nav_rpc` 的额外依赖位于：

```bash
pip install -r src/chassis_semantic/semantic_object_nav_rpc/requirements.txt
```

`bson_rpc` 需要 `bsonrpc2`、`rclpy`、`nav2_msgs`、`std_msgs` 等依赖；如使用 Conda 环境，请确保 ROS 2 Python 包路径可被当前解释器访问。

## 快速开始

### 1. 初始化 ROS 2 环境

```bash
source /opt/ros/jazzy/setup.bash
```

如需通过 `rosdep` 安装系统依赖，可在工作空间根目录执行：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> 如果存在未发布到 rosdep 的第三方包（例如特定版本的 Livox/Orbbec 驱动或模型推理依赖），请按对应项目文档手动安装。

### 2. 构建工作空间

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 检查和修改本机路径

仓库中部分脚本/launch 文件包含示例绝对路径，请按实际环境修改后再运行：

- `tum_dataset_generator.sh`：TUM 数据集输出路径。
- `pyslam.sh`：Conda 环境名称和 pySLAM 主程序路径。
- `remote_qwen.sh`：远程主机、模型路径、vLLM 参数。
- `src/chassis_semantic/semantic_object_nav_rpc/launch/semantic_object_nav_rpc.launch.py`：默认语义地图 `npz_path` / `ply_path`。
- `calibration.sh`：外部 LiDAR-Camera 标定工作空间路径。

## 常用流程

### 建图流程

```bash
source install/setup.bash
ros2 launch bringup slam.launch.py
```

或使用脚本：

```bash
./mapping.sh
```

保存 2D 地图：

```bash
./save_map.sh
```

默认会进入 `src/chassis_slam_and_nav/chassis_nav/maps` 并保存为 `room`。

### 最终导航流程

```bash
source install/setup.bash
ros2 launch bringup 2D_navigation_wheel_only.py
```

或使用脚本：

```bash
./nav_final.sh
```

该流程会按延时顺序启动底盘、LiDAR、轮速里程计、点云过滤、Nav2、点云地图匹配/NDT 定位，以及导航 RPC/语义导航 RPC。

### LiDAR-Camera 标定

```bash
./calibration.sh
```

标定相关结果和资源可参考 `src/bringup/calibration_res` 与 `src/chassis_semantic/lidar_camera_manual_calib`。

### Orbbec 深度彩色对齐

```bash
./orbbec.sh
```

或直接运行：

```bash
ros2 launch align_depth2color launch_alignment.py
```

### 导出 pySLAM TUM 数据集

```bash
./tum_dataset_generator.sh
```

运行前请确认脚本中的 `dataset_dir`、里程计话题等参数符合本机环境。

### 运行 pySLAM

```bash
./pyslam.sh
```

脚本默认激活 `pyslam` Conda 环境并运行 `lib/pyslam/main_slam.py --headless`。

### 语义分割与目标导航

```bash
./yoloe_seg.sh
```

该脚本默认激活 `itemid` Conda 环境，启动 YOLOE 分割节点，并连接本机导航 RPC 服务。

### 语义对象导航 RPC

```bash
source install/setup.bash
ros2 launch semantic_object_nav_rpc semantic_object_nav_rpc.launch.py \
  npz_path:=/path/to/semantic_dense_map_latest.npz \
  ply_path:=/path/to/semantic_dense_map_latest.ply \
  semantic_rpc_port:=6001 \
  nav_rpc_port:=6000
```

或使用脚本：

```bash
./semantic_nav_rpc_server.sh
```

## RPC 与 SDK

### 通用导航 RPC 服务

Python BSON-RPC 服务：

```bash
./nav_rpc_server.sh
```

C++ `nav_sdk` 服务：

```bash
ros2 launch nav_sdk nav_rpc_server.py
```

SDK 测试节点：

```bash
./nav_sdk.sh
```

### 语义目标 RPC 服务

`semantic_object_nav_rpc` 会读取 pySLAM 生成的语义地图文件，筛选语义对象，计算接近点，并调用导航 RPC 端口发送 Nav2 目标。默认端口：

- `6000`：导航 RPC 服务。
- `6001`：语义对象 RPC 服务。

## 可视化与调试

查看 TF 树：

```bash
./view_frames.sh
```

RViz 配置主要位于：

- `src/bringup/rviz/rviz.rviz`
- `src/chassis_slam_and_nav/chassis_nav/rviz/rviz.rviz`
- `src/chassis_slam_and_nav/pc_to_scan_and_slam/rviz/rviz.rviz`
- `src/robot_description/chassis_description/rviz/urdf_config.rviz`

## 配置文件提示

- Nav2 参数：`src/chassis_slam_and_nav/chassis_nav/config/nav2_params.yaml`
- 默认地图：`src/chassis_slam_and_nav/chassis_nav/maps/room.yaml`
- LIO-SAM MID360 参数：`src/chassis_slam_and_nav/lio_sam_mid360/config/params.yaml`
- 点云转 LaserScan/过滤参数：`src/chassis_slam_and_nav/pc_to_scan_and_slam/config/`
- EKF 参数：`src/chassis_slam_and_nav/ekf_filter/config/ekf_params.yaml`
- 相机到 LiDAR 静态 TF：可在 `src/bringup/launch/2D_navigation_wheel_only.py` 中调整。

## 注意事项

1. **硬件依赖强**：launch 文件默认依赖 Livox、Orbbec、底盘串口/驱动、Nav2 和本机 TF。没有硬件时部分节点会启动失败。
2. **路径需要本地化**：多个脚本中存在 `/home/robot2/...`、远端 `192.168.5.37` 等示例路径/主机，请按实际部署修改。
3. **Conda 与 ROS 2 Python 混用**：在 Conda 环境中运行 ROS 2 Python 节点时，需确保 `source /opt/ros/jazzy/setup.bash` 和 `source install/setup.bash` 后的 Python 路径正确。
4. **地图与坐标系一致性**：语义地图、Nav2 地图、定位输出和目标点均应使用一致的 `map`/`odom`/`base_link`/传感器坐标系。
5. **运行顺序**：建议先确保底盘、传感器、TF、定位、Nav2 均正常，再启动语义 RPC 和目标导航。

## 推荐开发命令

```bash
# 构建全部包
colcon build --symlink-install

# 只构建某个包
colcon build --symlink-install --packages-select semantic_object_nav_rpc

# 运行测试（如包中已有测试）
colcon test --packages-select semantic_object_nav_rpc
colcon test-result --verbose

# 查看 ROS 2 包是否可发现
source install/setup.bash
ros2 pkg list | grep semantic_object_nav_rpc
```

## 后续建议

- 将脚本中的绝对路径改为 launch 参数或环境变量。
- 为各个关键节点补充参数说明和话题接口表。
- 清理带有 `copy` / `Copy` 的历史文件，避免维护混淆。
- 为硬件启动、仿真启动、离线地图处理分别提供独立 launch/profile。
- 增加 CI，用于检查 Python 格式、C++ 编译和 ROS 2 包依赖。
