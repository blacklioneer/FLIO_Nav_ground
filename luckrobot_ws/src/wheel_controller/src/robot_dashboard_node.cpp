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
                if (msg->level < rcl_interfaces::msg::Log::INFO) { return; }
                
                std::lock_guard<std::mutex> lock(data_mutex_);
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

        init_pair(1, COLOR_BLACK, COLOR_WHITE);   
        init_pair(2, COLOR_GREEN, COLOR_WHITE);   
        init_pair(3, COLOR_RED, COLOR_WHITE);     
        init_pair(4, COLOR_BLUE, COLOR_WHITE);    
        init_pair(5, COLOR_MAGENTA, COLOR_WHITE); 
        init_pair(6, COLOR_RED, COLOR_WHITE);     

        bkgd(COLOR_PAIR(1)); 
    }

    // 🔥 新增 visual_width 参数，用于突破 C++ 中文长度计算限制，实现绝对视觉居中
    void draw_box(int y, int x, int height, int width, const std::string& title, int visual_width) {
        attron(COLOR_PAIR(4));
        // 画垂直和水平线
        mvvline(y + 1, x, ACS_VLINE, height - 2);
        mvvline(y + 1, x + width - 1, ACS_VLINE, height - 2);
        mvhline(y, x + 1, ACS_HLINE, width - 2);
        mvhline(y + height - 1, x + 1, ACS_HLINE, width - 2);
        // 画四个角
        mvaddch(y, x, ACS_ULCORNER);
        mvaddch(y, x + width - 1, ACS_URCORNER);
        mvaddch(y + height - 1, x, ACS_LLCORNER);
        mvaddch(y + height - 1, x + width - 1, ACS_LRCORNER);
        
        attron(A_BOLD);
        // 计算绝对居中的 X 坐标
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

        // 居中大标题 (视觉宽度约 45)
        attron(COLOR_PAIR(4) | A_BOLD | A_UNDERLINE);
        mvprintw(start_y, start_x + 25, "🚀 LUCK-ROBOT SYSTEM DASHBOARD (10.0.0.50) 🚀");
        attroff(COLOR_PAIR(4) | A_BOLD | A_UNDERLINE);

        // ========================================================
        // 模块 1：导航调度 (传入视觉宽度 13 进行绝对居中)
        // ========================================================
        draw_box(start_y + 2, start_x, 8, 31, " 🗺️ 导航任务 ", 13);
        
        attron(A_BOLD | COLOR_PAIR(1));
        mvprintw(start_y + 4, start_x + 2, "🎯 当前目标:");
        
        std::string display_task = current_task_;
        if(display_task.length() > 26) display_task = display_task.substr(0, 23) + "...";
        
        attron(COLOR_PAIR(5));
        mvprintw(start_y + 6, start_x + 2, "%s", display_task.c_str());
        attroff(COLOR_PAIR(5));
        attroff(A_BOLD | COLOR_PAIR(1));

        // ========================================================
        // 模块 2：丝杠高度 (传入视觉宽度 13 进行绝对居中)
        // ========================================================
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

        // ========================================================
        // 模块 3：轮侧雷达 (传入视觉宽度 13 进行绝对居中)
        // ========================================================
        draw_box(start_y + 2, start_x + 63, 8, 32, " 🛡️ 轮侧雷达 ", 13);
        attron(A_BOLD | COLOR_PAIR(1));
        
        // --- 左轮 ---
        mvprintw(start_y + 4, start_x + 65, "L:");
        if (left_dist_ >= 64.9) {
            attron(COLOR_PAIR(2)); printw(" ≥65.0 mm [✅]"); attroff(COLOR_PAIR(2));
        } else if (left_danger_) {
            attron(COLOR_PAIR(3)); printw("%5.1f mm [🚨]", left_dist_); attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(2)); printw("%5.1f mm [✅]", left_dist_); attroff(COLOR_PAIR(2));
        }

        // --- 右轮 ---
        mvprintw(start_y + 6, start_x + 65, "R:");
        if (right_dist_ >= 64.9) {
            attron(COLOR_PAIR(2)); printw(" ≥65.0 mm [✅]"); attroff(COLOR_PAIR(2));
        } else if (right_danger_) {
            attron(COLOR_PAIR(3)); printw("%5.1f mm [🚨]", right_dist_); attroff(COLOR_PAIR(3));
        } else {
            attron(COLOR_PAIR(2)); printw("%5.1f mm [✅]", right_dist_); attroff(COLOR_PAIR(2));
        }
        attroff(A_BOLD | COLOR_PAIR(1));

        // ========================================================
        // 模块 4：系统日志 (传入视觉宽度 31 进行绝对居中)
        // ========================================================
        draw_box(start_y + 10, start_x, 14, 95, " 📜 System Logs / 系统全局日志 ", 31);
        
        int log_y = start_y + 12;
        for (const auto& log : sys_logs_) {
            mvprintw(log_y, start_x + 2, ">");
            if (log.first >= rcl_interfaces::msg::Log::ERROR) {
                attron(COLOR_PAIR(3) | A_BOLD); 
                mvprintw(log_y, start_x + 4, "❌ %s", log.second.c_str());
                attroff(COLOR_PAIR(3) | A_BOLD);
            } else if (log.first >= rcl_interfaces::msg::Log::WARN) {
                attron(COLOR_PAIR(6) | A_BOLD); 
                mvprintw(log_y, start_x + 4, "⚠️ %s", log.second.c_str());
                attroff(COLOR_PAIR(6) | A_BOLD);
            } else {
                attron(COLOR_PAIR(1)); 
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