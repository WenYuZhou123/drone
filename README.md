# drone

基于 ROS1 `catkin` 的无人机综合工作区，核心方向是：

- PX4 / MAVROS 飞控对接
- 视觉与激光外部定位
- 目标检测、目标跟踪、精准降落、投放任务
- 激光建图、定位与导航
- 多机规划与仿真

这个仓库不是单一功能包，而是一整套用于无人机实飞、比赛任务和算法验证的工作区。

## 目录结构

```text
fly_ws/
├── src/             # 主要源码
├── _disabled_pkgs/  # 暂时停用的包
├── build/           # catkin 编译产物（已忽略）
└── devel/           # catkin 开发产物（已忽略）
```

## 主要能力

### 1. 飞控与基础控制

- `px4_command`：PX4 控制相关节点
- `offboard_single_position`：起飞后定点悬停
- `offboard_multi_position`：简单多点航迹飞行
- `self_check`：电池、遥控、MAVROS、里程计自检

### 2. 视觉 / 激光定位

- `vision_to_mavros`：把 T265 / TF / VIO 位姿转换成 MAVROS 可用外部定位
- `lidar_to_mavros`：把激光定位结果发送给飞控
- `FAST_LIO`：激光惯导里程计与建图
- `slam_navigation`：基于点云地图的定位
- `navigation` / `ros_navigation`：导航栈适配

### 3. 感知与目标识别

- `yolov8_ros`：YOLOv8 ROS 包
- `darknet_ros`：YOLOv4 / Darknet ROS 包
- `object_position`：结合 D435 深度图计算目标三维位置
- `robot_vision`：相机与视觉相关启动配置
- `ar_track_alvar`：AR Tag 识别

### 4. 任务级应用

- `precise_drop`：视觉识别 + 舵机投放 + 自动降落
- `v2_ar_track_landing`：基于 AR Tag 的视觉对准降落
- `target_follow` / `follow_yolov8` / `simple_follower`：目标跟随
- `complete_mission`：完整任务流程编排
- `robocup2024`：比赛任务逻辑

### 5. 多机与仿真

- `ego_swarm`：多无人机规划、环境建模、仿真与桥接模块

## 常用启动入口

主要入口集中在 `src/robot_bringup/launch/`：

- `bringup_offboard_single_position.launch`
  使用 T265 + MAVROS 完成起飞与悬停
- `bringup_v2_ar_track_landing.launch`
  使用 AR Tag 完成视觉降落
- `bringup_precise_drop.launch`
  使用相机识别完成投放任务
- `bringup_mid360.launch`
  使用 Livox MID360 + FAST_LIO + MAVROS
- `navigation_mid360.launch`
  使用 MID360 完成定位与导航
- `yolov8_d435.launch`
  启动 D435 和 YOLOv8 检测

## 依赖环境

典型环境：

- Ubuntu 18.04 / 20.04
- ROS Melodic / Noetic
- MAVROS
- PX4 或兼容飞控
- Intel RealSense T265 / D435
- Livox MID360
- OpenCV / PCL / Eigen
- PyTorch 与 YOLOv8 相关 Python 依赖

不同功能包的具体依赖不完全相同，建议按需要逐包安装。

## 构建

在工作区根目录执行：

```bash
catkin_make
source devel/setup.bash
```

如果使用 Livox、RealSense、YOLO 等模块，需要先完成对应驱动和 Python 依赖安装。

## 说明

- `build/`、`devel/`、本地编辑器配置和临时文件不纳入版本控制
- 某些超大地图、临时编译产物和无关缓存已经从 Git 历史中清理
- `_disabled_pkgs/` 里保留了一些旧版本或临时停用模块，默认不参与主流程

## 建议阅读顺序

如果你第一次看这个仓库，建议按下面顺序进入：

1. `src/robot_bringup/launch/`
2. `src/px4_command/`
3. `src/vision_to_mavros/`
4. `src/object_position/`
5. `src/precise_drop/` / `src/robocup2024/`
6. `src/FAST_LIO/` / `src/slam_navigation/`

这样能先看清主链路，再看算法和任务编排。
