import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions
# 导入条件判断工具
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('robot_display')
    default_model_path = os.path.join(urdf_tutorial_path, 'urdf', '0robot.SLDASM.urdf')
    
    # 1. 恢复 RViz 配置文件路径
    default_rviz_config_path = os.path.join(urdf_tutorial_path, 'config', 'display_model.rviz')
    
    # ================= 声明 Launch 参数 =================
    # 声明 model 路径参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
        
    # 声明 use_rviz2 参数，默认值为 'false'，防止与上层 Launch 冲突
    action_declare_use_rviz2 = launch.actions.DeclareLaunchArgument(
        name='use_rviz2', default_value='false',
        description='是否启动当前节点的 RViz2 (true/false)')
    # ====================================================

    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', LaunchConfiguration('model')]),
        value_type=str)
        
    # 状态发布节点 (负责解析 URDF 并发布 base_link 等 TF 坐标关系)
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 关节状态发布节点 (目前处于注释状态，按需开启)
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    # )
    
    # 2. 恢复 RViz 节点，并将其启动条件绑定到 use_rviz2
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        # 🔥 [核心修改] 只有当上层传入 use_rviz2:=true 时，才拉起这个特定的 RViz
        condition=IfCondition(LaunchConfiguration('use_rviz2'))
    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_use_rviz2,      # 注册新参数 use_rviz2
        #joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node                      # 附带条件判断的 RViz 节点
    ])