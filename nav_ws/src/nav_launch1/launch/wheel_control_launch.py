#!/usr/bin/env python3
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    target_pkg = "wheel_controller"
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    respawn = LaunchConfiguration("respawn", default="true")
    shutdown_timeout = "2.0"

    # ==================== 1. 丝杠电机节点 ====================
    lead_screw_motor_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="lead_screw_motor_node",
        name="lead_screw_motor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        respawn=respawn,
        respawn_delay=2.0,
        sigterm_timeout=shutdown_timeout,
        sigkill_timeout=shutdown_timeout
    )

    # ==================== 2. 底盘控制节点 ====================
    wheel_controller_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="wheel_controller_node",
        name="wheel_controller_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        respawn=respawn,
        respawn_delay=2.0,
        sigterm_timeout=shutdown_timeout,
        sigkill_timeout=shutdown_timeout
    )

    # ==================== 3. 危险指令节点 ====================
    danger_command_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="danger_command_node",
        name="danger_command_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        respawn=respawn,
        respawn_delay=2.0,
        sigterm_timeout=shutdown_timeout,
        sigkill_timeout=shutdown_timeout
    )

    # ==================== 4. 状态发布节点 ====================
    state_publisher_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="state_publisher_node",
        name="state_publisher_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        sigterm_timeout=shutdown_timeout,
        sigkill_timeout=shutdown_timeout
    )

    # ==================== 终极防崩溃：绝对时间轴延迟启动 ====================
    # 使用绝对时间轴，防止因为某个节点崩溃重启，导致连锁触发后续节点重复启动的 Bug

    delay_wheel_controller = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg=">>> T+3s: 拉起底盘控制节点 <<<"),
            wheel_controller_node
        ]
    )

    delay_danger_command = TimerAction(
        period=5.0, # 3秒 + 2秒
        actions=[
            LogInfo(msg=">>> T+5s: 拉起危险报警节点 <<<"),
            danger_command_node
        ]
    )

    delay_state_publisher = TimerAction(
        period=7.0, # 5秒 + 2秒
        actions=[
            LogInfo(msg=">>> T+7s: 拉起状态发布节点 <<<"),
            state_publisher_node
        ]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulation time for wheel_control nodes. Set true only when /clock is available."
        ),
        DeclareLaunchArgument(
            "respawn",
            default_value="true",
            description="Respawn hardware nodes after runtime crashes. Set false for clean Ctrl+C shutdown/debugging."
        ),
        LogInfo(msg=">>> 开始底层硬件节点顺序拉起序列 (绝对时间轴模式) <<<"),
        lead_screw_motor_node,   # T=0s 启动
        delay_wheel_controller,  # T=3s 启动
        delay_danger_command,    # T=5s 启动
        delay_state_publisher    # T=7s 启动
    ])
