cmake_minimum_required(VERSION 3.8)
project(wheel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== 寻找依赖包 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rcl_interfaces REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 导航调度节点需要的库
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# ==================== 编译节点可执行文件 ====================
add_executable(wheel_controller_node src/wheel_controller_node.cpp)
add_executable(lead_screw_motor_node src/lead_screw_motor_node.cpp)
add_executable(danger_command_node src/danger_command_node.cpp)
add_executable(state_publisher_node src/state_publisher_node.cpp)

# 导航调度与看板节点
add_executable(nav_manager_node src/nav_manager_node.cpp)
add_executable(robot_dashboard_node src/robot_dashboard_node.cpp)


# ==================== 链接库与依赖项 ====================
ament_target_dependencies(state_publisher_node rclcpp std_msgs sensor_msgs)
ament_target_dependencies(danger_command_node rclcpp std_msgs)
ament_target_dependencies(wheel_controller_node rclcpp geometry_msgs std_msgs)

# 丝杠节点专属配置 (libserial)
ament_target_dependencies(lead_screw_motor_node rclcpp std_msgs)
target_include_directories(lead_screw_motor_node PUBLIC ${LIBSERIAL_INCLUDE_DIRS})
target_link_directories(lead_screw_motor_node PUBLIC ${LIBSERIAL_LIBRARY_DIRS})
target_link_libraries(lead_screw_motor_node ${LIBSERIAL_LIBRARIES})

# 导航调度节点专属配置
ament_target_dependencies(nav_manager_node
  rclcpp
  std_msgs
  rclcpp_action
  nav2_msgs
)

# 机器人本地看板节点专属配置 (链接 ncursesw 以支持中文 UTF-8！)
ament_target_dependencies(robot_dashboard_node rclcpp std_msgs)
target_link_libraries(robot_dashboard_node ncursesw)
# 机器人本地看板节点专属配置 (链接 Ncurses 和 Log接口)
ament_target_dependencies(robot_dashboard_node rclcpp std_msgs rcl_interfaces)
target_link_libraries(robot_dashboard_node ncursesw)

# ==================== 安装规则 ====================
install(
  TARGETS 
    wheel_controller_node 
    lead_screw_motor_node 
    danger_command_node 
    state_publisher_node
    nav_manager_node      
    robot_dashboard_node   
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()