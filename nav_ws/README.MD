# Nav2 导航相关包说明

本文档说明 `nav_ws/src` 下 Nav2 相关包的职责、接口和关键配置：

- `nav2_nav`：Nav2 启动、地图加载、导航参数配置。
- `nav2_custom_planner`：自定义全局规划器插件。
- `nav2_custom_controller`：自定义局部控制器插件。
- `nav2_traversability_layer`：面向足式机器人的 PointCloud2 可通行性 costmap layer。

总启动入口在 `nav_launch1/launch/nav_bringup.launch.py`。本文只聚焦 Nav2 导航相关链路；底层 wheel、丝杠等硬件控制不是 Nav2 包的必要组成部分。

## 启动链路

完整系统启动顺序由 `nav_launch1/launch/nav_bringup.launch.py` 编排。和 Nav2 导航直接相关的关键阶段如下：

1. T+4s 启动 Livox 雷达驱动。
2. T+42s 启动 `open3d_loc/launch/localization_3d_g1.launch.py`，提供定位/里程计相关数据。
3. T+60s 启动 `nav2_nav/launch/nav2.launch.py`，拉起 Nav2。
4. T+74s 启动导航任务调度节点，向 Nav2 的 `navigate_to_pose` action 发送目标点。

`wheel_control_launch.py` 中的 wheel、丝杠、危险检测、状态发布属于底层硬件/业务节点，不属于 Nav2 导航包本身。若当前部署的速度下发不需要控制底层 wheel 和丝杠，可以在实际 bringup 中不启动或移除相关 Include/Node。

Nav2 只要求最终存在一个速度消费方订阅 `cmd_vel`，这个消费方可以是实际底盘驱动、仿真接口、桥接节点或其他控制模块，不强依赖 `wheel_controller_node`。

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
| Action | `navigate_to_pose` | 导航任务调度节点发送目标点 |
| Topic 输入 | `/Odometry_loc` | Nav2 行为树和速度平滑器使用的里程计 |
| Topic 输入 | `/cloud_registered_body_1` | traversability layer 使用的机体系 3D 点云 |
| Topic 输入 | `/scan_2d` | 备用 LaserScan 障碍输入，当前不作为足式避障主链路 |
| Topic 输出 | `cmd_vel` | Nav2 输出的速度命令，由实际部署的底盘/仿真/桥接模块消费 |
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

注意：`/Odometry_loc` 需要由定位/里程计模块提供，当前 Nav2 配置不从底层 wheel 或丝杠节点获取里程计。

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
plugins: ["traversability_layer", "inflation_layer"]
```

局部 costmap 订阅 FAST-LIO 输出的机体系点云：

```yaml
traversability_layer:
  plugin: "nav2_traversability_layer::TraversabilityLayer"
  enabled: True
  pointcloud_topic: /cloud_registered_body_1
  max_step_height: 0.24
  max_slope_traversable: 38.0
  reset_each_update: True
```

作用：

- 订阅 `/cloud_registered_body_1`。
- 按栅格估计地面高度、障碍高度、障碍比例、邻域高差和坡度。
- 对可跨越台阶/可通行坡面输出软代价。
- 对超过足式机器人能力的台阶、陡坡、密集高障碍输出硬障碍。
- 通过 `inflation_layer` 给硬障碍膨胀安全距离。

### 全局 costmap

全局地图配置在 `global_costmap.global_costmap.ros__parameters`：

```yaml
global_frame: map
robot_base_frame: base_link
resolution: 0.05
track_unknown_space: true
plugins: ["static_layer", "traversability_layer", "inflation_layer"]
```

全局 costmap 同时使用：

- `static_layer`：订阅 `map_server` 发布的静态 PGM 地图。
- `traversability_layer`：订阅 `/cloud_registered_body_1`，把实时地形代价/硬障碍加入全局 costmap。
- `inflation_layer`：障碍膨胀。

这意味着全局规划不仅看静态 PGM 地图，也会受实时 3D 点云地形代价影响。当前全局 traversability layer 配置为 `reset_each_update: False`，会累积已观测区域的代价；如果动态障碍残留明显，应改为带衰减机制或仅在局部 costmap 使用实时层。

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

## nav2_traversability_layer

`nav2_traversability_layer` 是面向足式机器人的 Nav2 costmap layer 插件。

插件导出文件：

```xml
nav2_traversability_layer/traversability_layer.xml
```

插件名：

```yaml
nav2_traversability_layer::TraversabilityLayer
```

它直接订阅 FAST-LIO 输出的机体系点云：

```yaml
pointcloud_topic: /cloud_registered_body_1
```

数据流：

```text
/cloud_registered_body_1
  -> nav2_traversability_layer
  -> local_costmap / global_costmap
  -> MPPI / 全局规划器
```

### 功能

该 layer 不再把 3D 点云简单压成 `LaserScan`，而是按 costmap 栅格统计 2.5D 地形特征：

- `ground_z`：栅格地面高度估计。
- `obstacle_height`：地面上方障碍高度。
- `obstacle_ratio`：超过可跨越高度的点占障碍候选点比例。
- `height_diff`：邻域地面最大高度差。
- `slope`：邻域坡度。

然后根据足式机器人能力输出代价：

- 平整可走区域输出低代价。
- 接近最大台阶高度、坡度阈值的区域输出软代价。
- 超过 `hard_obstacle_height`、`hard_step_height`、`hard_slope_limit` 的区域输出硬障碍。

### 关键参数

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `pointcloud_topic` | `/cloud_registered_body_1` | 输入机体系 3D 点云 |
| `max_step_height` | `0.24` | 可跨越台阶/高差阈值 |
| `max_slope_traversable` | `38.0` | 可通行坡度阈值，单位 deg |
| `robot_body_height` | `1.2` | 地面上方障碍检测高度范围 |
| `obstacle_min_height` | `0.06` | 高于地面多少开始视为障碍候选 |
| `hard_obstacle_height` | `0.35` | 超过后直接视为硬障碍 |
| `hard_slope_limit` | `50.0` | 超过后直接视为硬障碍 |
| `soft_cost_max` | `200` | 可通行但不理想区域的最高软代价 |
| `reset_each_update` | local 为 `True`，global 为 `False` | 局部重算，全局累积已观测代价 |

### local/global 使用方式

局部 costmap：

```yaml
plugins: ["traversability_layer", "inflation_layer"]
traversability_layer:
  plugin: "nav2_traversability_layer::TraversabilityLayer"
  pointcloud_topic: /cloud_registered_body_1
  reset_each_update: True
```

全局 costmap：

```yaml
plugins: ["static_layer", "traversability_layer", "inflation_layer"]
traversability_layer:
  plugin: "nav2_traversability_layer::TraversabilityLayer"
  pointcloud_topic: /cloud_registered_body_1
  reset_each_update: False
```

局部使用 `reset_each_update: True`，避免动态障碍残留。全局使用 `reset_each_update: False`，让全局规划器能看到已观测区域的实时更新；如果全局动态障碍残留影响规划，应增加衰减机制或把全局 layer 改为只处理硬障碍。

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

这是 Nav2 局部控制器插件，不直接订阅点云或 `/scan_2d`，也不直接发布 `cmd_vel`。它通过 `nav2_core::Controller` 接口被 `controller_server` 调用：

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

之后由 `controller_server` 和 `velocity_smoother` 处理成 `cmd_vel`。`cmd_vel` 是 Nav2 对外的速度输出接口，具体由哪个节点订阅取决于实际部署：

- 真机部署时，一般由底盘驱动或控制桥接节点订阅。
- 仿真部署时，一般由仿真底盘插件或仿真控制器订阅。
- 当前如果不需要 wheel 和丝杠控制，可以不把 `cmd_vel` 绑定到 `wheel_controller_node`。

## 当前避障能力判断

当前系统支持 3D 点云进入 costmap：

- `local_costmap` 通过 `nav2_traversability_layer` 订阅 `/cloud_registered_body_1`。
- `global_costmap` 通过 `nav2_traversability_layer` 订阅 `/cloud_registered_body_1`。
- 该 layer 同时输出台阶/坡度软代价和硬障碍代价。
- 全局规划器从 `global_costmap` 读代价，因此重新规划时会考虑实时地形更新。

但是当前 `nav2_custom_controller` 仍然不是真正的采样式局部避障器。它的 `computeVelocityCommands()` 里没有读取 local costmap 做碰撞检测、轨迹采样或速度优化，只是沿路径转向和直行。若要充分利用 traversability costmap，建议将局部控制器替换为 MPPI。

因此动态避障能力可以概括为：

- 静态可通行性地图避障：支持。
- 3D 点云地形代价写入 local/global costmap：支持。
- 全局重新规划绕障/绕不可通行地形：支持。
- 当前自定义局部控制器实时绕障：能力较弱。
- MPPI 接入后，可更好利用连续代价场做局部避障。

## 更换局部规划器为 MPPI

可以更换为 MPPI，前提是系统安装了 `nav2_mppi_controller`，并且 Nav2 版本支持对应插件。

MPPI 作为 Nav2 controller 插件，替换后仍然通过 Nav2 的 `cmd_vel` 输出速度命令。实际速度下发只需要保证有合适的消费方订阅 `cmd_vel`，不要求接入 wheel 或丝杠节点。

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
2. `/cloud_registered_body_1` 的 frame 必须能通过 TF 转到 `odom`/`map`。
3. `map -> odom -> base_link` TF 必须连续。
4. `cmd_vel` 输出频率要和实际速度消费方的控制频率匹配。
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
| 点云话题 | `traversability_layer.pointcloud_topic` | `/cloud_registered_body_1` |
| 可跨越高度 | `traversability_layer.max_step_height` | `0.24m` |
| 可通行坡度 | `traversability_layer.max_slope_traversable` | `38deg` |
| 硬障碍高度 | `traversability_layer.hard_obstacle_height` | `0.35m` |
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

查看 traversability 输入点云：

```bash
ros2 topic hz /cloud_registered_body_1
```

备用：查看 LaserScan：

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
