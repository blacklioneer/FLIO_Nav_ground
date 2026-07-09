#!/bin/bash

# 1. 延迟 5 秒，等待系统桌面加载完毕
sleep 5

# 2. 加载 ROS 2 环境和你的工作空间
source /opt/ros/humble/setup.bash
source /home/nvidia/nav/nav_ws/install/setup.bash

# 3. 设置相同的 Domain ID 确保能收到数据,启动shm
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/nav/fastdds_shm.xml

# 4. 启动看板节点
ros2 run wheel_controller robot_dashboard_node
