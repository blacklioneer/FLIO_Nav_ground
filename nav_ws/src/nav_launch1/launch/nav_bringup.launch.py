from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz1 = LaunchConfiguration("use_rviz1", default="True")
    start_robot_display = LaunchConfiguration("start_robot_display", default="false")
    start_lidar_driver = LaunchConfiguration("start_lidar_driver", default="false")
    start_localization = LaunchConfiguration("start_localization", default="false")

    # ==========================================
    # 阶段 1：可选显示/TF 节点
    # ==========================================
    robot_display_event = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg="============ [事件 1] T+1s：拉起 Robot Display ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('robot_display'), 'launch', 'robot_display.launch.py'])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time
                }.items()
            )
        ],
        condition=IfCondition(start_robot_display)
    )

    # ==========================================
    # 阶段 2：可选雷达驱动；现阶段默认不启动，外部已启动雷达时保持 false
    # ==========================================
    livox_driver_event = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="============ [事件 2] T+2s：拉起 Livox 雷达驱动 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py'])
                )
            )
        ],
        condition=IfCondition(start_lidar_driver)
    )

    # ==========================================
    # 阶段 3：可选定位
    # ==========================================
    open3d_event = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="============ [事件 3] T+3s：拉起 Open3D 定位 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('open3d_loc'), 'launch', 'localization_3d_g1.launch.py'])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time
                }.items()
            )
        ],
        condition=IfCondition(start_localization)
    )

    # ==========================================
    # 阶段 4：Nav2，只负责规划、控制并发布 /cmd_vel
    # ==========================================
    nav2_event = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="============ [事件 4] T+8s：拉起 Nav2，输出 /cmd_vel ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('nav2_nav'), 'launch', 'nav2.launch.py'])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_rviz1": use_rviz1
                }.items()
            )
        ]
    )

    # ==========================================
    # 组装返回
    # ==========================================
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Single clock switch for this bringup. Use true when /clock is available."
        ),
        DeclareLaunchArgument(
            "use_rviz1",
            default_value="True",
            description="Whether to start RViz through nav2_nav."
        ),
        DeclareLaunchArgument(
            "start_robot_display",
            default_value="false",
            description="Start robot_display for model/static TF visualization. Default false when simulation already publishes robot TF."
        ),
        DeclareLaunchArgument(
            "start_lidar_driver",
            default_value="false",
            description="Start Livox lidar driver. Default false because current stage uses externally started or simulated lidar."
        ),
        DeclareLaunchArgument(
            "start_localization",
            default_value="false",
            description="Start Open3D localization. Default false because current workflow starts localization externally."
        ),
        LogInfo(msg="============ [系统启动] 轻量导航模式：不启动 wheel_control，只发布 /cmd_vel ============"),
        robot_display_event,
        livox_driver_event,
        open3d_event,
        nav2_event
    ])
