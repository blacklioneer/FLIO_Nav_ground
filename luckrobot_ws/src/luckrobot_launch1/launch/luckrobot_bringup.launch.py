import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_ros.actions  
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ==========================================
    # 阶段 1：底层硬件与传感器 (事件级错峰启动)
    # ==========================================
    
    wheel_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('luckrobot_launch1'), 'launch', 'wheel_control_launch.py'])
        )
    )

    robot_display_event = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="============ [事件 2] T+2s：拉起 Robot Display ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('robot_display'), 'launch', 'robot_display.launch.py'])
                )
            )
        ]
    )

    livox_driver_event = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="============ [事件 3] T+4s：拉起 Livox 雷达驱动 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py'])
                )
            )
        ]
    )

    # ==========================================
    # 阶段 2：发布移动指令 (T+29s)
    # ==========================================
    
    pub_lead_screw_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/lead_screw/displacement', 
            'std_msgs/msg/Float32', '{data: -462.0}', '--once'
        ],
        output='screen'
    )

    pub_event = TimerAction(
        period=29.0,
        actions=[
            LogInfo(msg="============ [事件 4] T+29s：下发下降指令 (强行等待 10 秒后启动上层算法) ============"),
            pub_lead_screw_cmd
        ]
    )

    # ==========================================
    # 阶段 3 & 4 & 5 & 6：高阶算法与宏观控制错峰拉起 (纯定时模式)
    # ==========================================
    
    # 29秒 + 10秒 = 39秒
    open3d_event = TimerAction(
        period=39.0,
        actions=[
            LogInfo(msg="============ [事件 5] T+39s：丝杠下降已过10秒！全功率拉起 Open3D ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('open3d_loc'), 'launch', 'localization_3d_g1.launch.py'])
                )
            )
        ]
    )

    # 39秒 + 20秒 = 59秒
    nav2_event = TimerAction(
        period=59.0,
        actions=[
            LogInfo(msg="============ [事件 6] T+59s：Open3D缓冲期结束！平稳拉起 Nav2 导航栈 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('nav2_luckrobot'), 'launch', 'nav2.launch.py'])
                )
            )
        ]
    )

    # 39秒 + 35秒 = 74秒
    nav_manager_event = TimerAction(
        period=74.0,
        actions=[
            LogInfo(msg="============ [事件 7] T+74s：Nav2 就绪！拉起中央导航调度节点 ============"),
            launch_ros.actions.Node(
                package="wheel_controller",
                executable="nav_manager_node",
                name="nav_manager_node",
                output="screen",
                parameters=[{"use_sim_time": False}]
            )
        ]
    )

    # 39秒 + 40秒 = 79秒
    voice_cmd_event = TimerAction(
        period=79.0,
        actions=[
            LogInfo(msg="============ [事件 8] T+79s：打通最后一块 VLA 拼图，启动语音节点 ============"),
            launch_ros.actions.Node(
                package="voice_controller",
                executable="voice_cmd_node",
                name="voice_cmd_node",
                output="screen",
                parameters=[{"use_sim_time": False}]
            )
        ]
    )

    # 🔥 39秒 + 43秒 = 82秒：真正的大轴！全部启动完毕后，下发开机语音！
    # 0x0C 对应的十进制是 12
    startup_voice_event = TimerAction(
        period=82.0,
        actions=[
            LogInfo(msg="============ [事件 9] T+82s：全部系统就绪！下发开机完毕语音播报 📢 ============"),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '/voice_prompt', 
                    'std_msgs/msg/UInt8', '{data: 12}', '--once'
                ],
                output='screen'
            )
        ]
    )

    # ==========================================
    # 组装返回
    # ==========================================
    return LaunchDescription([
        LogInfo(msg="============ [系统启动] 纯定时无阻塞流水线事件引擎已启动 ============"),
        LogInfo(msg="============ [事件 1] T+0s：立即触发：拉起 Wheel Control ============"),
        wheel_control_launch,
        robot_display_event,
        livox_driver_event,
        pub_event,
        open3d_event,
        nav2_event,
        nav_manager_event,
        voice_cmd_event,
        startup_voice_event  
    ])