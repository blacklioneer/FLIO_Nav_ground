#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/log.hpp> 
#include <ncurses.h>
#include <locale.h>
#include <mutex>
#include <string>
#include <chrono>
#include <deque>

class RobotDashboardNode : public rclcpp::Node {
public:
    RobotDashboardNode() : Node("robot_dashboard_node") {
        
        screw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lead_screw/current_position", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                screw_pos_ = msg->data;
            });

        danger_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "obstacle_distances", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (msg->data.size() >= 4) {
                    left_dist_ = msg->data[0];
                    right_dist_ = msg->data[1];
                    left_danger_ = (msg->data[2] == 1.0);
                    right_danger_ = (msg->data[3] == 1.0);
                }
            });

        task_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/vla_task_trigger", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_task_ = msg->data;
            });

        rosout_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", 20,
            [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
                // 忽略 DEBUG 级别日志
                if (msg->level < rcl_interfaces::msg::Log::INFO) { return; }
                
                std::lock_guard<std::mutex> lock(data_mutex_);

                // ==========================================
                // 🔥 智能日志过滤器：限频与警报穿透
                // ==========================================
                bool is_wheel_pkg = (msg->name.find("wheel_controller") != std::string::npos || 
                                     msg->name.find("lead_screw") != std::string::npos ||
                                     msg->name.find("danger_command") != std::string::npos);

                // 只有当它是底层硬件的 INFO 日志时，才进行 10 秒限频拦截
                if (is_wheel_pkg && msg->level == rcl_interfaces::msg::Log::INFO) {
                    auto now = this->now();
                    if ((now - last_wheel_log_time_).seconds() < 10.0) {
                        return; // 丢弃频繁的底层运行日志，保持屏幕清爽
                    }
                    last_wheel_log_time_ = now;
                }
                // 注意：一旦爆出 WARN 或 ERROR，它会无视上面的 if，直接穿透显示！
                // ==========================================

                std::string log_str = "[" + msg->name + "] " + msg->msg;
                if (log_str.length() > 86) {
                    log_str = log_str.substr(0, 83) + "...";
                }
                
                sys_logs_.push_back({msg->level, log_str});
                if (sys_logs_.size() > 11) {
                    sys_logs_.pop_front();
                }
            });

        init_ui();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotDashboardNode::update_ui, this));
    }

    ~RobotDashboardNode() {
        endwin(); 
    }

private:
    std::mutex data_mutex_;
    float screw_pos_ = 0.0;
    bool left_danger_ = false;
    bool right_danger_ = false;
    float left_dist_ = 65.0;  
    float right_dist_ = 65.0;
    std::string current_task_ = "等待调度指令...";
    
    rclcpp::Time last_wheel_log_time_{0, 0, RCL_ROS_TIME}; // 用于限频记录时间
    std::deque<std::pair<int8_t, std::string>> sys_logs_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr screw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr danger_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_ui() {
        setlocale(LC_ALL, ""); 
        initscr();            
        noecho();             
        cbreak();             
        curs_set(0);          
        start_color();        

        // 白底亮色主题调色板
        init_pair(1, COLOR_BLACK, COLOR_WHITE);   // INFO 黑色
        init_pair(2, COLOR_GREEN, COLOR_WHITE);   // 成功 绿色
        init_pair(3, COLOR_RED, COLOR_WHITE);     // ERROR 红色
        init_pair(4, COLOR_BLUE, COLOR_WHITE);    // 边框 蓝色
        init_pair(5, COLOR_MAGENTA, COLOR_WHITE); // 强调 紫色
        init_pair(6, COLOR_MAGENTA, COLOR_WHITE); // WARN 警告(在白底上用紫色代替黄色最醒目)

        bkgd(COLOR_PAIR(1)); 
    }

    void draw_box(int y, int x, int height, int width, const std::string& title, int visual_width) {
        attron(COLOR_PAIR(4));
        mvvline(y + 1, x, ACS_VLINE, height - 2);
        mvvline(y + 1, x + width - 1, ACS_VLINE, height - 2);
        mvhline(y, x + 1, ACS_HLINE, width - 2);
        mvhline(y + height - 1, x + 1, ACS_HLINE, width - 2);
        mvaddch(y, x, ACS_ULCORNER);
        mvaddch(y, x + width - 1, ACS_URCORNER);
        mvaddch(y + height - 1, x, ACS_LLCORNER);
        mvaddch(y + height - 1, x + width - 1, ACS_LRCORNER);
        
        attron(A_BOLD);
        int title_x = x + (width - visual_width) / 2;
        if (title_x < x + 1) { title_x = x + 1; }
        mvprintw(y, title_x, "%s", title.c_str());
        attroff(A_BOLD);
        attroff(COLOR_PAIR(4));
    }

    void update_ui() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        erase(); 

        int max_y, max_x;
        getmaxyx(stdscr, max_y, max_x);
        
        int layout_width = 95;  
        int layout_height = 24; 
        int start_y = (max_y - layout_height) / 2;
        int start_x = (max_x - layout_width) / 2;
        if (start_y < 0) { start_y = 0; }
        if (start_x < 0) { start_x = 0; }

        attron(COLOR_PAIR(4) | A_BOLD | A_UNDERLINE);
        mvprintw(start_y, start_x + 25, "🚀 LUCK-ROBOT SYSTEM DASHBOARD (10.0.0.50) 🚀");
        attroff(COLOR_PAIR(4) | A_BOLD | A_UNDERLINE);

        // ==================== 1：导航调度 ====================
        draw_box(start_y + 2, start_x, 8, 31, " 🗺️ 导航任务 ", 13);
        attron(A_BOLD | COLOR_PAIR(1));
        mvprintw(start_y + 4, start_x + 2, "🎯 当前目标:");
        std::string display_task = current_task_;
        if(display_task.length() > 26) display_task = display_task.substr(0, 23) + "...";
        attron(COLOR_PAIR(5));
        mvprintw(start_y + 6, start_x + 2, "%s", display_task.c_str());
        attroff(COLOR_PAIR(5));
        attroff(A_BOLD | COLOR_PAIR(1));

        // ==================== 2：丝杠高度 ====================
        draw_box(start_y + 2, start_x + 32, 8, 31, " ⚙️ 丝杠高度 ", 13);
        float abs_height = screw_pos_ + 662.0f;
        attron(A_BOLD | COLOR_PAIR(1));
        mvprintw(start_y + 4, start_x + 34, "📏 相对 Z:");
        attron(COLOR_PAIR(5)); printw("%6.1f mm", screw_pos_); attroff(COLOR_PAIR(5));
        mvprintw(start_y + 5, start_x + 34, "📐 离地 H:");
        attron(COLOR_PAIR(2)); printw("%6.1f mm", abs_height); attroff(COLOR_PAIR(2));

        int screw_bar = (int)((-screw_pos_ / 462.0) * 12);
        if (screw_bar < 0) { screw_bar = 0; }
        if (screw_bar > 12) { screw_bar = 12; }
        
        mvprintw(start_y + 7, start_x + 34, "行程[");
        attron(COLOR_PAIR(4));
        for(int i=0; i<12; i++) { printw(i < screw_bar ? "■" : " "); }
        attroff(COLOR_PAIR(4));
        printw("]");
        attroff(A_BOLD | COLOR_PAIR(1));

        // ==================== 3：轮侧雷达 ====================
        draw_box(start_y + 2, start_x + 63, 8, 32, " 🛡️ 轮侧雷达 ", 13);
        attron(A_BOLD | COLOR_PAIR(1));
        
        mvprintw(start_y + 4, start_x + 65, "L:");
        if (left_dist_ >= 64.9) {
            attron(COLOR_PAIR(2)); printw(" ≥65.0 mm [✅]"); attroff(COLOR_PAIR(2));
        } else if (left_danger_) {
            attron(COLOR_PAIR(3)); printw("%5.1f mm [🚨]", left_dist_); attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(2)); printw("%5.1f mm [✅]", left_dist_); attroff(COLOR_PAIR(2));
        }

        mvprintw(start_y + 6, start_x + 65, "R:");
        if (right_dist_ >= 64.9) {
            attron(COLOR_PAIR(2)); printw(" ≥65.0 mm [✅]"); attroff(COLOR_PAIR(2));
        } else if (right_danger_) {
            attron(COLOR_PAIR(3)); printw("%5.1f mm [🚨]", right_dist_); attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(2)); printw("%5.1f mm [✅]", right_dist_); attroff(COLOR_PAIR(2));
        }
        attroff(A_BOLD | COLOR_PAIR(1));

        // ==================== 4：系统日志 ====================
        draw_box(start_y + 10, start_x, 14, 95, " 📜 System Logs / 系统全局日志 ", 31);
        
        int log_y = start_y + 12;
        for (const auto& log : sys_logs_) {
            mvprintw(log_y, start_x + 2, ">");
            
            if (log.first >= rcl_interfaces::msg::Log::FATAL) {
                attron(COLOR_PAIR(3) | A_BOLD | A_UNDERLINE | A_BLINK); 
                mvprintw(log_y, start_x + 4, "💀 %s", log.second.c_str());
                attroff(COLOR_PAIR(3) | A_BOLD | A_UNDERLINE | A_BLINK);
            } else if (log.first >= rcl_interfaces::msg::Log::ERROR) {
                attron(COLOR_PAIR(3) | A_BOLD); 
                mvprintw(log_y, start_x + 4, "❌ %s", log.second.c_str());
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else if (log.first >= rcl_interfaces::msg::Log::WARN) {
                attron(COLOR_PAIR(6) | A_BOLD); // 紫色高亮
                mvprintw(log_y, start_x + 4, "⚠️ %s", log.second.c_str());
                attroff(COLOR_PAIR(6) | A_BOLD);
            } else {
                attron(COLOR_PAIR(1)); // 黑色常规
                mvprintw(log_y, start_x + 4, "💬 %s", log.second.c_str());
                attroff(COLOR_PAIR(1));
            }
            log_y++;
        }

        refresh();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDashboardNode>());
    rclcpp::shutdown();
    return 0;
}