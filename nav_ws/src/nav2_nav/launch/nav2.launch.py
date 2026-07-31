import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # 获取目录
    nav2_nav_dir = get_package_share_directory('nav2_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    rviz_config_dir = os.path.join(nav2_nav_dir, 'rviz', 'my_nav2_view.rviz')
    
    # 统一时钟开关：仿真/Gazebo 使用 true，真机且无 /clock 时使用 false。
    # 该参数会下发给 map_server、Nav2 组件、RViz，并通过 navigation_launch.py 覆盖参数文件。
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    # 【新增】：控制是否启动 RViz 的开关，默认设置为 'False'
    use_rviz1 = launch.substitutions.LaunchConfiguration('use_rviz1', default='False')
    log_level = launch.substitutions.LaunchConfiguration('log_level', default='info')
    
    # 读取你通过 pcd2pgm 保存的干净的 2D 栅格地图
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(nav2_nav_dir, 'maps', 'test_map.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(nav2_nav_dir, 'config', 'nav2_params.yaml'))
    shutdown_timeout = '4.0'

    return launch.LaunchDescription([
        # =========================================================
        # 0. 声明 Launch 参数，方便外部调用时修改
        # =========================================================
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='true',
                                             description='Single clock switch for Nav2, RViz, map_server and path timestamps. Use true with /clock; false on real robot without /clock.'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),
        # 声明 use_rviz1 参数，明确告诉用户这个参数的作用
        launch.actions.DeclareLaunchArgument('use_rviz1', default_value='False',
                                             description='Whether to start RViz2 on the robot (default: False for headless Jetson)'),
        launch.actions.DeclareLaunchArgument('log_level', default_value='info',
                                             description='Logging level for Nav2 component container, e.g. info or debug'),
        launch.actions.RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    launch.actions.LogInfo(msg='Shutting down nav2_nav launch: canceling Nav2 actions and publishing zero velocity.'),
                    launch.actions.ExecuteProcess(
                        cmd=['ros2', 'run', 'nav2_nav', 'nav2_shutdown_guard.py', '--once'],
                        output='screen'
                    )
                ]
            )
        ),
        # Launch 级统一设置；各 Node 仍显式传入该参数，避免组合式启动或外部 include 漏掉。
        SetParameter(name='use_sim_time', value=use_sim_time),

        # =========================================================
        # 0.5. 退出保护：Ctrl+C 时取消 Nav2 action 并发布零速度
        # =========================================================
        Node(
            package='nav2_nav',
            executable='nav2_shutdown_guard.py',
            name='nav2_shutdown_guard',
            output='screen',
            sigterm_timeout=shutdown_timeout,
            sigkill_timeout=shutdown_timeout
        ),

        # =========================================================
        # 1. 单独启动 Map Server (提供全局 2D 代价底图)
        # =========================================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': use_sim_time}],
            sigterm_timeout=shutdown_timeout,
            sigkill_timeout=shutdown_timeout
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}],
            sigterm_timeout=shutdown_timeout,
            sigkill_timeout=shutdown_timeout
        ),

        # =========================================================
        # 2.5. 启动 Nav2 组件容器
        # =========================================================
        # MPPI 在当前 Humble + 非组合式 navigation_launch 路径下触发了
        # controller_server / local_costmap 的 executor 冲突。
        # 这里切回 Nav2 官方更稳定的 composed bringup 方式。
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            output='screen',
            parameters=[nav2_param_path, {'autostart': True, 'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            sigterm_timeout=shutdown_timeout,
            sigkill_timeout=shutdown_timeout
        ),

        # =========================================================
        # 3. 启动核心导航层 (完全绕过 localization_launch 和 AMCL)
        # =========================================================
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'use_composition': 'True',
                'container_name': 'nav2_container',
                'use_respawn': 'False'}.items(),
        ),

        # =========================================================
        # 4. 启动 RViz2 (带有条件判断：只有 use_rviz1:=True 时才启动)
        # =========================================================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            sigterm_timeout=shutdown_timeout,
            sigkill_timeout=shutdown_timeout,
            # 条件锁！如果没传 use_rviz1:=True，不会运行
            condition=IfCondition(use_rviz1)
        ),
    ])
