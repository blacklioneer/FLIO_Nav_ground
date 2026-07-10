# Nav2 导航相关包说明

本文档说明 `nav_ws/src` 下三个 Nav2 相关包的职责、接口和关键配置：

- `nav2_nav`：Nav2 启动、地图加载、导航参数配置。
- `nav2_custom_planner`：自定义全局规划器插件。
- `nav2_custom_controller`：自定义局部控制器插件。

总启动入口在 `nav_launch1/launch/nav_bringup.launch.py`。其中 `wheel_control_launch.py` 只负责启动底层硬件节点，不直接启动 Nav2。

## 启动链路

完整系统启动顺序由 `nav_launch1/launch/nav_bringup.launch.py` 编排：

1. T+0s 启动 `nav_launch1/launch/wheel_control_launch.py`。
2. T+2s 启动 `robot_display`。
3. T+4s 启动 Livox 雷达驱动。
4. T+42s 启动 `open3d_loc/launch/localization_3d_g1.launch.py`。
5. T+60s 启动 `nav2_nav/launch/nav2.launch.py`。
6. T+74s 启动 `wheel_controller/nav_manager_node`。
7. T+79s 启动语音节点。

`wheel_control_launch.py` 启动的底层节点：

| 节点 | 包 | 作用 |
| --- | --- | --- |
| `lead_screw_motor_node` | `wheel_controller` | 丝杠电机控制 |
| `wheel_controller_node` | `wheel_controller` | 底盘速度控制，订阅 `cmd_vel` |
| `danger_command_node` | `wheel_controller` | 危险/报警相关逻辑 |
| `state_publisher_node` | `wheel_controller` | 状态发布 |

## nav2_nav

`nav2_nav` 是 Nav2 的启动和配置包，核心文件：

- `nav2_nav/launch/nav2.launch.py`
- `nav2_nav/config/nav2_params.yaml`
- `nav2_nav/maps/test_map.yaml`
- `nav2_nav/maps/test_map.pgm`
- `nav2_nav/rviz/my_nav2_view.rviz`

### 地图加载

`nav2_nav/launch/nav2.launch.py` 单独启动 `nav2_map_server/map_server`：

```python
Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': use_sim_time}]
)
```

默认地图参数：

```yaml
map: nav2_nav/maps/test_map.yaml
```

`test_map.yaml` 内容：

```yaml
image: test_map.pgm
mode: trinary
resolution: 0.05
origin: [-4.44, -55.3, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

也就是说，全局 PGM 地图由 `map_server` 从 `test_map.yaml` 加载，实际图像文件是 `test_map.pgm`。

### Nav2 启动方式

`nav2.launch.py` 绕过 `localization_launch.py` 和 AMCL，直接 include：

```python
nav2_bringup/launch/navigation_launch.py
```

这说明当前定位不是 AMCL，而是依赖外部定位系统提供 `map -> odom -> base_link` 相关 TF 和 `/Odometry_loc` 里程计。结合总启动链路看，这部分大概率由 `open3d_loc` 提供。

### 主要接口

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| Action | `navigate_to_pose` | `nav_manager_node` 发送导航目标 |
| Topic 输入 | `/Odometry_loc` | Nav2 行为树和速度平滑器使用的里程计 |
| Topic 输入 | `/scan_2d` | 局部/全局 costmap 的 LaserScan 障碍输入 |
| Topic 输出 | `cmd_vel` | 最终底盘速度命令，`wheel_controller_node` 订阅 |
| Map | `test_map.yaml` / `test_map.pgm` | 全局静态地图 |
| TF | `map`, `odom`, `base_link` | Nav2 使用的坐标系 |

### 里程计配置

`nav2_params.yaml` 中有两处使用 `/Odometry_loc`：

```yaml
bt_navigator:
  ros__parameters:
    odom_topic: /Odometry_loc
```

```yaml
velocity_smoother:
  ros__parameters:
    odom_topic: "/Odometry_loc"
```

注意：`wheel_controller_node` 当前没有发布 `/Odometry_loc`，它主要订阅 `cmd_vel` 并控制电机。因此 `/Odometry_loc` 需要由定位/里程计模块提供。

### 局部 costmap

局部地图配置在 `local_costmap.local_costmap.ros__parameters`：

```yaml
global_frame: odom
robot_base_frame: base_link
rolling_window: true
width: 3
height: 3
resolution: 0.05
robot_radius: 0.28
plugins: ["obstacle_layer", "inflation_layer"]
```

局部 costmap 订阅雷达：

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan_2d
    data_type: "LaserScan"
    clearing: True
    marking: True
    raytrace_max_range: 3.0
    obstacle_max_range: 2.5
```

作用：

- 订阅 `/scan_2d`。
- 将雷达障碍标记进局部 costmap。
- 用 raytrace 清除已经不再被观测到的障碍。
- 通过 `inflation_layer` 给障碍物膨胀安全距离。

### 全局 costmap

全局地图配置在 `global_costmap.global_costmap.ros__parameters`：

```yaml
global_frame: map
robot_base_frame: base_link
resolution: 0.05
track_unknown_space: true
plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

全局 costmap 同时使用：

- `static_layer`：订阅 `map_server` 发布的静态 PGM 地图。
- `obstacle_layer`：订阅 `/scan_2d`，把实时障碍加入全局 costmap。
- `inflation_layer`：障碍膨胀。

这意味着全局规划不仅看静态地图，也会受 `/scan_2d` 当前观测障碍影响。

### 行为树与恢复行为

`bt_navigator` 使用 `map` 作为全局坐标系，`base_link` 作为机器人坐标系：

```yaml
global_frame: map
robot_base_frame: base_link
bt_loop_duration: 100
```

`behavior_server` 配置了这些恢复/行为插件：

```yaml
behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
```

行为服务器使用局部 costmap：

```yaml
costmap_topic: local_costmap/costmap_raw
footprint_topic: local_costmap/published_footprint
global_frame: odom
robot_base_frame: base_link
```

## nav2_custom_planner

`nav2_custom_planner` 是全局规划器插件包。

插件导出文件：

```xml
nav2_custom_planner/custom_planner_plugin.xml
```

插件名：

```yaml
nav2_custom_planner/CustomPlanner
```

在 `nav2_params.yaml` 中启用：

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_custom_planner/CustomPlanner"
      interpolation_resolution: 0.1
```

### 功能

实现文件：

```text
nav2_custom_planner/src/nav2_custom_planner.cpp
```

主要逻辑：

- 从 `planner_server` 提供的 `global_costmap` 读取代价地图。
- 将起点和终点从世界坐标转换为地图栅格。
- 使用类似 A* 的搜索生成路径。
- 使用六邻域扩展 `getHexNeighbors()`。
- 避开障碍、未知区域和高代价区域。
- 对高 cost 栅格施加安全惩罚。
- 对转向施加惩罚，让路径更偏向长直线。
- 使用视线检测做路径平滑。
- 按 `interpolation_resolution` 插值输出路径。

关键安全判定：

```cpp
bool CustomPlanner::isNodeValid(unsigned int x, unsigned int y) {
  unsigned char cost = costmap_->getCost(x, y);
  return (cost < 253 && cost != nav2_costmap_2d::NO_INFORMATION);
}
```

视线平滑更严格：

```cpp
bool CustomPlanner::isSightValid(unsigned int x, unsigned int y) {
  unsigned char cost = costmap_->getCost(x, y);
  return (cost < 180 && cost != nav2_costmap_2d::NO_INFORMATION);
}
```

### 接口

这是 Nav2 插件，不直接订阅普通 ROS 话题。它通过 `nav2_core::GlobalPlanner` 接口被 `planner_server` 调用：

```cpp
nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal)
```

输入：

- 当前起点 `start`
- 目标点 `goal`
- `planner_server` 注入的 `global_costmap`
- TF buffer

输出：

- `nav_msgs/msg/Path`

依赖的坐标系：

- `start.header.frame_id` 必须等于 global costmap frame，当前是 `map`。
- `goal.header.frame_id` 必须等于 global costmap frame，当前是 `map`。

## nav2_custom_controller

`nav2_custom_controller` 是局部控制器插件包。

插件导出文件：

```xml
nav2_custom_controller/nav2_custom_controller.xml
```

插件名：

```yaml
nav2_custom_controller::CustomController
```

在 `nav2_params.yaml` 中启用：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_custom_controller::CustomController"
      max_linear_speed: 0.10
      max_angular_speed: 0.70
      lookahead_dist: 0.80
```

### 功能

实现文件：

```text
nav2_custom_controller/src/custom_controller.cpp
```

主要逻辑：

- 接收全局路径 `setPlan()`。
- 每次 `computeVelocityCommands()` 根据机器人当前位姿找路径上的前视目标点。
- 如果角度偏差大，原地旋转。
- 如果角度偏差小，直线前进。
- 接近终点后，使用终点 yaw 做最后姿态对准。

控制参数：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `max_linear_speed` | `0.10` | 最大前进速度，单位 m/s |
| `max_angular_speed` | `0.70` | 最大角速度，单位 rad/s |
| `lookahead_dist` | `0.80` | 路径前视距离，单位 m |
| `controller_frequency` | `10.0` | 控制器运行频率 |

目标判定：

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.20
  yaw_goal_tolerance: 0.20
```

进度检查：

```yaml
progress_checker:
  required_movement_radius: 0.05
  movement_time_allowance: 45.0
```

### 接口

这是 Nav2 局部控制器插件，不直接订阅 `/scan_2d`，也不直接发布 `cmd_vel`。它通过 `nav2_core::Controller` 接口被 `controller_server` 调用：

```cpp
void setPlan(const nav_msgs::msg::Path &path)
```

```cpp
geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * goal_checker)
```

输入：

- 当前机器人位姿 `pose`
- 当前速度 `velocity`
- 全局路径 `global_plan_`
- TF buffer

输出：

- `geometry_msgs/msg/TwistStamped`

之后由 `controller_server` 和 `velocity_smoother` 处理成底盘 `cmd_vel`。底盘侧 `wheel_controller_node` 订阅 `cmd_vel`：

```cpp
this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&WheelControllerNode::cmd_vel_callback, this, std::placeholders::_1));
```

## 当前避障能力判断

当前系统支持动态障碍进入 costmap：

- `local_costmap` 订阅 `/scan_2d`。
- `global_costmap` 也订阅 `/scan_2d`。
- `clearing: True` 和 `marking: True` 都已开启。
- 全局规划器从 costmap 读代价，因此重新规划时会考虑实时障碍。

但是当前局部控制器不是真正的采样式局部避障器。`nav2_custom_controller` 的 `computeVelocityCommands()` 里没有读取 local costmap 做碰撞检测、轨迹采样或速度优化，只是沿路径转向和直行。

因此动态避障能力可以概括为：

- 静态地图避障：支持。
- 雷达障碍写入 costmap：支持。
- 全局重新规划绕障：支持。
- 局部控制器实时绕开突然出现的动态障碍：能力较弱。
- 对行人等动态障碍的平滑绕行：不如 MPPI、DWB、TEB、RPP 等局部规划器。

## 更换局部规划器为 MPPI

可以更换为 MPPI，前提是系统安装了 `nav2_mppi_controller`，并且 Nav2 版本支持对应插件。

当前底盘控制链路订阅 `cmd_vel`，MPPI 作为 Nav2 controller 插件输出速度命令后仍可接入现有 `wheel_controller_node`。

### 配置替换位置

修改：

```text
nav2_nav/config/nav2_params.yaml
```

把当前控制器：

```yaml
controller_plugins: ["FollowPath"]

FollowPath:
  plugin: "nav2_custom_controller::CustomController"
  max_linear_speed: 0.10
  max_angular_speed: 0.70
  lookahead_dist: 0.80
```

替换为 MPPI 控制器配置。示例：

```yaml
controller_plugins: ["FollowPath"]

FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  time_steps: 40
  model_dt: 0.05
  batch_size: 1000
  vx_std: 0.2
  wz_std: 0.4
  vx_max: 0.20
  vx_min: -0.01
  vy_max: 0.0
  wz_max: 0.8
  motion_model: "DiffDrive"
  visualize: false
```

实际使用时还需要补齐 MPPI critics，例如：

- `ConstraintCritic`
- `ObstaclesCritic`
- `GoalCritic`
- `GoalAngleCritic`
- `PathAlignCritic`
- `PathFollowCritic`
- `PathAngleCritic`
- `PreferForwardCritic`

建议先使用保守速度：

```yaml
vx_max: 0.10
wz_max: 0.70
```

确认车体、TF、costmap 和雷达数据稳定后，再逐步增大速度和调整 critics 权重。

### 替换 MPPI 后的注意事项

1. `local_costmap` 必须稳定更新，否则 MPPI 无法可靠避障。
2. `/scan_2d` 的 frame 必须能通过 TF 转到 `base_link`/`odom`。
3. `map -> odom -> base_link` TF 必须连续。
4. `cmd_vel` 输出频率和底盘控制频率要匹配。
5. 当前 `velocity_smoother.feedback` 是 `OPEN_LOOP`，如果底盘里程计稳定，可评估改成闭环反馈。
6. MPPI 比当前自定义控制器更吃 CPU，Jetson 上要控制 `batch_size`、`time_steps` 和控制频率。

## 关键参数速查

| 功能 | 配置位置 | 当前值 |
| --- | --- | --- |
| 全局坐标系 | `bt_navigator.global_frame` | `map` |
| 机器人坐标系 | `bt_navigator.robot_base_frame` | `base_link` |
| 里程计话题 | `bt_navigator.odom_topic` | `/Odometry_loc` |
| 地图文件 | `nav2.launch.py` 默认 `map` | `maps/test_map.yaml` |
| 局部 costmap frame | `local_costmap.global_frame` | `odom` |
| 全局 costmap frame | `global_costmap.global_frame` | `map` |
| 雷达话题 | `obstacle_layer.scan.topic` | `/scan_2d` |
| 雷达类型 | `obstacle_layer.scan.data_type` | `LaserScan` |
| 局部地图大小 | `local_costmap.width/height` | `3m x 3m` |
| 地图分辨率 | `resolution` | `0.05m` |
| 机器人半径 | `robot_radius` | `0.28m` |
| 膨胀半径 | `inflation_radius` | `0.35m` |
| 全局规划器 | `planner_server.GridBased.plugin` | `nav2_custom_planner/CustomPlanner` |
| 局部控制器 | `controller_server.FollowPath.plugin` | `nav2_custom_controller::CustomController` |
| 控制频率 | `controller_frequency` | `10Hz` |
| 最大线速度 | `FollowPath.max_linear_speed` | `0.10m/s` |
| 最大角速度 | `FollowPath.max_angular_speed` | `0.70rad/s` |
| 速度平滑里程计 | `velocity_smoother.odom_topic` | `/Odometry_loc` |

## 调试命令

查看地图：

```bash
ros2 topic echo /map
```

查看雷达：

```bash
ros2 topic echo /scan_2d
```

查看里程计：

```bash
ros2 topic echo /Odometry_loc
```

查看速度命令：

```bash
ros2 topic echo /cmd_vel
```

查看 TF：

```bash
ros2 run tf2_tools view_frames
```

单独启动 Nav2：

```bash
ros2 launch nav2_nav nav2.launch.py
```

带 RViz 启动：

```bash
ros2 launch nav2_nav nav2.launch.py use_rviz1:=True
```

指定地图启动：

```bash
ros2 launch nav2_nav nav2.launch.py map:=/absolute/path/to/map.yaml
```
