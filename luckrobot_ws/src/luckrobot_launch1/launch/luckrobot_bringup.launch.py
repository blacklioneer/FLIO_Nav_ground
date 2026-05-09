import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
import launch_ros.actions  # 🔥 引入 Node 动作
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ==========================================
    # 阶段 1：底层硬件与传感器 (事件级错峰启动)
    # ==========================================
    
    # [事件 1] T=0s，立即拉起底盘控制(这个Launch里面包含了丝杠、车轮、警报和状态)
    wheel_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('luckrobot_launch1'), 'launch', 'wheel_control_launch.py'])
        )
    )

    # [事件 2] T=2s，延时错峰拉起 Robot Display
    robot_display_event = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="============ [事件 2] 2秒错峰：拉起 Robot Display ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('robot_display'), 'launch', 'robot_display.launch.py'])
                )
            )
        ]
    )

    # [事件 3] T=4s，延时错峰拉起 Livox 雷达
    livox_driver_event = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="============ [事件 3] 4秒错峰：拉起 Livox 雷达驱动 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py'])
                )
            )
        ]
    )

    # ==========================================
    # 阶段 2：发布移动指令，并挂载"物理动作监听器"
    # ==========================================
    
    pub_lead_screw_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/lead_screw/displacement', 
            'std_msgs/msg/Float32', '{data: -462.0}', '--once'
        ],
        output='screen'
    )

    # 监听物理结束信号
    wait_motor_finish_cmd = ExecuteProcess(
        cmd=['ros2 topic echo /rosout | grep -m 1 -- "-462.0"'],
        shell=True,
        output='log'
    )

    pub_event = TimerAction(
        period=29.0,
        actions=[
            LogInfo(msg="============ [事件 4] 距离雷达启动已过 25s：下发下降指令并监听物理结束 ============"),
            pub_lead_screw_cmd,
            wait_motor_finish_cmd 
        ]
    )

    # ==========================================
    # 阶段 3 & 4 & 5：基于物理移动结束事件的高阶算法错峰拉起
    # ==========================================
    
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('open3d_loc'), 'launch', 'localization_3d_g1.launch.py'])
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_luckrobot'), 'launch', 'nav2.launch.py'])
        )
    )

    # 🔥 新增：中央导航调度节点
    nav_manager_node = launch_ros.actions.Node(
        package="wheel_controller",
        executable="nav_manager_node",
        name="nav_manager_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # 核心事件钩子：死死盯住物理移动结束信号
    open3d_and_nav2_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_motor_finish_cmd,  
            on_exit=[
                # 【事件 5】：瞬间拉起 Open3D (占用大量算力)
                LogInfo(msg="============ [事件 5] 物理移动结束！全功率拉起 Open3D ============"),
                open3d_loc_launch,
                
                # 【事件 6】：等待 20 秒 (避开 Open3D 算力峰值)，拉起 Nav2
                LogInfo(msg="============ [等待中] 开启 20 秒算力缓冲倒计时 ============"),
                TimerAction(
                    period=20.0,
                    actions=[
                        LogInfo(msg="============ [事件 6] 缓冲期结束！平稳拉起 Nav2 导航栈 ============"),
                        nav2_launch
                    ]
                ),

                # 【事件 7】：等待 35 秒 (20秒给Open3D + 15秒给Nav2启动Action服务)，压轴拉起中央调度节点！
                LogInfo(msg="============ [等待中] 开启 35 秒终极调度节点倒计时 ============"),
                TimerAction(
                    period=35.0,
                    actions=[
                        LogInfo(msg="============ [事件 7] Nav2 服务已全面就绪！压轴拉起中央导航调度节点 ============"),
                        nav_manager_node
                    ]
                )
            ]
        )
    )

    # ==========================================
    # 组装返回
    # ==========================================
    return LaunchDescription([
        LogInfo(msg="============ [系统启动] 物理级严格流水线事件引擎已启动 ============"),
        LogInfo(msg="============ [事件 1] 立即触发：拉起 Wheel Control ============"),
        wheel_control_launch,
        robot_display_event,
        livox_driver_event,
        pub_event,
        open3d_and_nav2_event_handler
    ])