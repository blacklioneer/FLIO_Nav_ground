# pcd2pgm

基于 ROS2 和 PCL 的离线 PCD/PLY 到 PGM/OccupancyGrid 转换工具。当前版本保留原有输入参数、发布 topic 和 `map_saver_cli` 保存流程，同时参考 traversability layer 的思路，增加面向足式/人形机器人的 2.5D 可通行性地图生成方式。

## 功能
- 注意！！！点云地图一定要做平面确认，即修改坐标系平面的z轴朝向，建议使用cloudcompare的平面工具。同时，拼接点云可能存在在一个z轴有2个值的现象，这会导致costmap的生成存在问题
- 读取指定 `.pcd` 或 `.ply` 点云文件。
- 发布滤波后的预览点云到 `pcd_cloud`。
- 发布 `nav_msgs/OccupancyGrid` 到配置的 `map_topic_name`，默认 `map`。
- `terrain_mode: true` 时按地面高度、障碍密度、坡度、高差和离线膨胀生成 PGM。
- `terrain_mode: false` 时保留原始高度切片占用栅格生成方式。

## 人形机器人地形模式

地形模式不再简单把某个 Z 高度切片内的点全部投影为障碍，而是按栅格做可通行性分析：

1. 在 `analysis_z_min` 到 `analysis_z_max` 范围内读取点云。
2. 每个栅格用 `ground_percentile` 估计地面高度，减少噪点影响。
3. 在地面上方 `robot_body_height` 范围内统计障碍候选比例，避免把高处天花板直接当作地面障碍。
4. 用邻域地面高度计算坡度和最大高度差。
5. 对小面积无观测栅格做反距离插值补全。
6. 按 OccupancyGrid 标准输出 `-1/0..100`：`-1` unknown，`0` free，`100` occupied，中间值为灰度代价。
7. 对占用栅格执行 `obstacle_inflation_radius` 离线膨胀，保存 PGM 的方式不变。

参考项目：https://github.com/ypat999/dog_slam/tree/main/LIO-SAM_MID360_ROS2_PKG/ros2/src/traversability_layer

## 编译

先确认安装了 Nav2 map server，保存 PGM 需要用到它：

```bash
sudo apt install ros-${ROS_DISTRO}-nav2-map-server
```

```bash
cd ~/FLIO_Nav_ground/nav_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --packages-select pcd2pgm
source install/setup.bash
```

## 运行

在 `config/pcd2pgm.yaml` 中设置输入点云：

```yaml
input_file: /home/blacklily/Documents/fused_clean.pcd
```

或：

```yaml
input_file: /home/blacklily/Documents/fused_clean.ply
```

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

确认 `/map` 已经发布：

```bash
ros2 topic echo /map --once
```

另开一个终端保存栅格地图，`-f` 后面写实际文件名前缀，不要带 `< >`：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/test_map
```

成功后会生成：

```text
~/maps/test_map.pgm
~/maps/test_map.yaml
```

上述命令默认订阅 `/map` topic。若修改了 `map_topic_name`，例如改成 `terrain_map`，保存时需要重映射：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/test_map --ros-args -r map:=/terrain_map
```

## 参数

默认配置位于 `config/pcd2pgm.yaml`，每个参数都已在 YAML 中注释。常用调参项如下：

| 参数 | 作用 |
| --- | --- |
| `input_file` | 输入点云路径，支持 `.pcd` 和 `.ply` |
| `terrain_mode` | 是否启用人形机器人可通行性 PGM 生成 |
| `analysis_z_min`, `analysis_z_max` | 地形分析使用的 Z 范围 |
| `ground_estimation_method` | 地面估计方式，`upper_densest` 适合过滤下方重影层 |
| `ground_percentile` | 地面高度估计分位数 |
| `ground_cluster_tolerance` | Z 聚类间隔，用于分离上下两层拼接重影 |
| `max_step_height` | 最大可跨越高度差，超过后标记占用 |
| `robot_body_height` | 地面上方障碍检测高度范围 |
| `obstacle_ratio_threshold` | 障碍密度阈值 |
| `max_slope_traversable` | 最大可通行坡度，单位 deg |
| `interp_search_radius` | 小洞插值半径，单位栅格 |
| `obstacle_inflation_radius` | 离线障碍膨胀半径，单位栅格 |
| `map_resolution` | 输出地图分辨率 |
| `map_topic_name` | OccupancyGrid 输出 topic，默认 `map` |

## 调参建议

- 室内有天花板误判时，增大 `robot_body_height` 不能解决问题，应确保 `analysis_z_max` 不低于机器人高度，但障碍判断主要依赖地面上方 `robot_body_height` 内的点。
- 地图过于保守时，适当增大 `max_step_height`、`max_slope_traversable`，或减小 `obstacle_inflation_radius`。
- 台阶/坡道被误判为可通行时，减小 `max_step_height` 或 `max_slope_traversable`。
- 小洞过多时，增大 `interp_search_radius`；若插值导致未知区域过多变为可通行，减小该值或增大 `min_interp_neighbors`。
- 多段拼接点云在地面下方出现重影层时，使用 `ground_estimation_method: upper_densest`，并把 `ground_cluster_tolerance` 设得略小于两层重影的 Z 间距。
