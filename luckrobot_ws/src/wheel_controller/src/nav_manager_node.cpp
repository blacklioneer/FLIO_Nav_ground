#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <map>
#include <string>
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 定义四自由度路点：底盘(X,Y,Yaw) + 丝杠高度(Z)
struct Waypoint {
    double x;
    double y;
    double yaw;         // 机器人的朝向弧度 (0 ~ 3.14)
    float screw_height; // 丝杠位移高度 (0.0 ~ -462.0)
};

class NavManagerNode : public rclcpp::Node {
public:
    NavManagerNode() : Node("nav_manager_node") {
        
        // ==========================================
        // 初始化 20 个目标点坐标字典
        // 你可以通过 `ros2 topic echo /goal_pose` 在 RViz2 里点选获取真实坐标
        // ==========================================
        target_locations_["厨房桌子"]   = { 2.5,  1.2,  1.57, -300.0};
        target_locations_["客厅茶几"]   = {-1.0,  3.5,  0.00, -250.0};
        target_locations_["充电桩"]     = { 0.0,  0.0,  0.00,    0.0};
        target_locations_["一号货架"]   = { 5.0, -2.0, -1.57, -400.0};
        target_locations_["二号货架"]   = { 5.0, -3.0, -1.57, -462.0};
        target_locations_["废料回收站"] = {-4.0, -4.0,  3.14, -100.0};
        target_locations_["检验台"]     = { 1.5,  4.2,  1.57, -200.0};
        target_locations_["包装区"]     = { 3.2, -1.5,  0.00, -350.0};
        target_locations_["工位A"]      = { 6.1,  0.5,  1.57, -150.0};
        target_locations_["工位B"]      = { 6.1,  2.5,  1.57, -150.0};
        target_locations_["工位C"]      = { 6.1,  4.5,  1.57, -150.0};
        target_locations_["工具架"]     = {-2.5,  1.0, -1.57, -450.0};
        target_locations_["会议室"]     = { 8.0,  8.0,  0.00, -220.0};
        target_locations_["前台"]       = { 0.0,  8.0,  3.14, -100.0};
        target_locations_["饮水机"]     = {-1.5, -2.5,  1.57, -380.0};
        target_locations_["书架顶层"]   = { 2.0,  2.0,  0.00, -460.0};
        target_locations_["书架底层"]   = { 2.0,  2.0,  0.00,  -50.0};
        target_locations_["沙发"]       = {-3.0,  2.0,  1.57, -120.0};
        target_locations_["阳台"]       = { 4.0,  7.0, -1.57, -200.0};
        target_locations_["垃圾桶"]     = { 0.5, -1.0,  3.14, -400.0};

        // 创建 Nav2 的动作客户端
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 创建丝杠下发话题
        screw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lead_screw/displacement", 10);

        // 创建大模型任务触发话题 (发给 10.0.0.51 VLA)
        vla_trigger_pub_ = this->create_publisher<std_msgs::msg::String>("/vla_task_trigger", 10);

        // 订阅语音文本
        voice_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/user_voice_cmd", 10,
            std::bind(&NavManagerNode::voice_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "🚀 中央调度节点启动成功！(已加载 20 个目标点预设)");
    }

private:
    std::map<std::string, Waypoint> target_locations_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr screw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vla_trigger_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_sub_;
    
    std::string current_task_name_;

    // Yaw(偏航角弧度) 转 四元数 辅助函数
    void yaw_to_quaternion(double yaw, double& qx, double& qy, double& qz, double& qw) {
        qx = 0.0; qy = 0.0;
        qz = std::sin(yaw / 2.0);
        qw = std::cos(yaw / 2.0);
    }

    void voice_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        RCLCPP_INFO(this->get_logger(), "🗣️ 收到文本/语音指令: [%s]", command.c_str());

        bool location_found = false;
        std::string target_name = "";
        Waypoint target_wp;

        // 1. 解析目标点
        for (const auto& location : target_locations_) {
            if (command.find(location.first) != std::string::npos) {
                location_found = true;
                target_name = location.first;
                target_wp = location.second;
                current_task_name_ = command; // 记录全句指令，后续发给VLA
                break; // 匹配到一个就退出
            }
        }

        // 2. 异常处理：没有匹配到目标点
        if (!location_found) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 未在指令中发现已知的预设地点！小车保持原地不动。");
            return;
        }

        // 3. 正常处理：匹配成功，开始下发动作
        RCLCPP_INFO(this->get_logger(), "🎯 成功解析目标地点: [%s]", target_name.c_str());
        RCLCPP_INFO(this->get_logger(), "   - Nav2 坐标 : X=%.2f, Y=%.2f, Yaw=%.2f", target_wp.x, target_wp.y, target_wp.yaw);
        RCLCPP_INFO(this->get_logger(), "   - 丝杠高度 : %.1f mm", target_wp.screw_height);
        
        // 3.1 同步下发丝杠动作 (底层已实现异步等待，这里安全发送)
        std_msgs::msg::Float32 screw_msg;
        screw_msg.data = target_wp.screw_height;
        screw_pub_->publish(screw_msg);

        // 3.2 调用 Nav2 执行底盘移动
        send_nav_goal(target_wp.x, target_wp.y, target_wp.yaw);
    }

    void send_nav_goal(double x, double y, double yaw) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "❌ Nav2 Action 服务器未就绪！无法导航！");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        double qx, qy, qz, qw;
        yaw_to_quaternion(yaw, qx, qy, qz, qw);
        goal_msg.pose.pose.orientation.x = qx;
        goal_msg.pose.pose.orientation.y = qy;
        goal_msg.pose.pose.orientation.z = qz;
        goal_msg.pose.pose.orientation.w = qw;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // 回调：当导航到达终点或被中断时触发
        send_goal_options.result_callback =
            [this](const GoalHandleNav::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "✅ 导航圆满到达目的地！丝杠已到位！");
                        RCLCPP_INFO(this->get_logger(), "📡 正在向 10.0.0.51 (VLA节点) 下发视觉作业指令...");
                        
                        // 目标点到达，将任务需求原文发送给大模型处理
                        {
                            std_msgs::msg::String vla_msg;
                            vla_msg.data = current_task_name_;
                            vla_trigger_pub_->publish(vla_msg);
                        }
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "❌ 导航被强行终止！(可能由于障碍物死锁)");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "⚠️ 导航任务被外部取消。");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "❌ 导航失败，未知原因！");
                        break;
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "🚙 路线已下发至 Nav2，机器人开始移动...");
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavManagerNode>());
    rclcpp::shutdown();
    return 0;
}