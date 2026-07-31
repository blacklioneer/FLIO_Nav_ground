from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # ==========================================
    # 1. 声明可以通过命令行传入的 launch 参数
    # ==========================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock simulation time for FAST-LIO, localization and RViz'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz2'
    )

    # 获取参数当前值，统一传给所有会发布/消费时间戳的节点
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # 获取包路径
    fast_lio_share = FindPackageShare('fast_lio')
    open3d_loc_share = FindPackageShare('open3d_loc')

    # 包含 fast_lio 的 launch 文件
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fast_lio_share,
                'launch',
                'mapping.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 包含 open3d_loc 的 launch 文件
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'open3d_loc_g1.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # RViz 节点配置
    rviz_config_path = PathJoinSubstitution([
        open3d_loc_share,
        'rviz_cfg',
        'fastlio.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_map_cur',
        arguments=['-d', rviz_config_path],
        output='screen',
        prefix='nice',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        # ==========================================
        # 2. RViz节点加上条件判断！
        # ==========================================
        condition=IfCondition(use_rviz)  # 只有当 use_rviz 为 'true' 时，才启动这个节点
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        fast_lio_launch,
        open3d_loc_launch,
        rviz_node
    ])
