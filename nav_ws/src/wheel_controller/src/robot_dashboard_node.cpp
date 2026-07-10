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
#include <algorithm>
#include <cctype>

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
            "/user_voice_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_task_ = msg->data;
            });

        // 🔥 核心优化：源头日志已纯净，彻底移除 10 秒过滤锁！
        // 现在任何节点的 INFO/WARN/ERROR 都会被丝滑、实时地显示出来
        rosout_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", 20,
            [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
                if (msg->level < rcl_interfaces::msg::Log::INFO) { return; }
                
                std::lock_guard<std::mutex> lock(data_mutex_);

                std::string log_str = "[" + msg->name + "] " + msg->msg;
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
        init_pair(6, COLOR_YELLOW, COLOR_WHITE);  

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

    void print_colorized_log(int y, int x, const std::string& prefix, int prefix_color, const std::string& log_line, int max_right_x) {
        move(y, x);
        attron(COLOR_PAIR(prefix_color) | A_BOLD);
        printw("%s ", prefix.c_str());
        attroff(COLOR_PAIR(prefix_color) | A_BOLD);

        std::string word = "";
        bool stop_printing = false;

        auto print_word = [&]() {
            if (word.empty() || stop_printing) { return; }
            
            std::string lw = word;
            std::transform(lw.begin(), lw.end(), lw.begin(), ::tolower);
            
            int color = 1; int attr = A_NORMAL;

            if (lw.find("error") != std::string::npos || lw.find("fail") != std::string::npos || 
                lw.find("fatal") != std::string::npos || lw == "false" || lw.find("timeout") != std::string::npos) {
                color = 3; attr = A_BOLD;
            } else if (lw.find("warn") != std::string::npos || lw.find("alert") != std::string::npos) {
                color = 6; attr = A_BOLD;
            } else if (lw.find("success") != std::string::npos || lw.find("ok") != std::string::npos || 
                       lw == "true" || lw.find("connect") != std::string::npos || lw == "up" || lw.find("start") != std::string::npos) {
                color = 2; attr = A_BOLD;
            } else if (lw == "ip" || lw == "return" || lw == "exit" || lw == "node" || 
                       lw == "topic" || lw == "port" || lw == "msg" || lw == "mac" || lw == "ros" || lw == "position" || lw == "sensor") {
                color = 5; attr = A_BOLD;
            } else if (std::any_of(lw.begin(), lw.end(), ::isdigit) || lw == "info" || lw == "data") {
                color = 4; attr = A_BOLD; 
            }

            if (attr != A_NORMAL) { attron(attr); }
            attron(COLOR_PAIR(color));
            
            for (char wc : word) {
                int cy, cx;
                getyx(stdscr, cy, cx);
                (void)cy; 
                if (cx >= max_right_x - 3) {
                    printw("...");
                    stop_printing = true;
                    break;
                }
                printw("%c", wc);
            }
            
            attroff(COLOR_PAIR(color));
            if (attr != A_NORMAL) { attroff(attr); }
            word.clear();
        };

        for (char c : log_line) {
            if (stop_printing) break;
            
            if (c == ' ' || c == '[' || c == ']' || c == ':' || c == ',' || c == '=' || 
                c == '(' || c == ')' || c == '/' || c == '-' || c == '>' || c == '<') {
                print_word();
                if (stop_printing) break;
                
                int cy, cx;
                getyx(stdscr, cy, cx);
                (void)cy; 
                if (cx >= max_right_x - 3) {
                    attron(COLOR_PAIR(1)); printw("..."); attroff(COLOR_PAIR(1));
                    stop_printing = true;
                    break;
                }
                attron(COLOR_PAIR(1)); printw("%c", c); attroff(COLOR_PAIR(1));
            } else {
                word += c;
            }
        }
        if (!stop_printing) print_word(); 
    }

    std::string safe_utf8_substr(const std::string& str, size_t max_bytes) {
        if (str.length() <= max_bytes) return str;
        size_t len = 0;
        while (len < max_bytes) {
            unsigned char c = str[len];
            size_t char_len = 1; 
            if ((c & 0xE0) == 0xC0) char_len = 2;
            else if ((c & 0xF0) == 0xE0) char_len = 3; 
            else if ((c & 0xF8) == 0xF0) char_len = 4;
            
            if (len + char_len > max_bytes) break; 
            len += char_len;
        }
        return str.substr(0, len) + "...";
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
        
        std::string display_task = safe_utf8_substr(current_task_, 39);
        
        attron(COLOR_PAIR(5));
        mvprintw(start_y + 6, start_x + 2, "%s", display_task.c_str());
        attroff(COLOR_PAIR(5));
        attroff(A_BOLD | COLOR_PAIR(1));

        // ==================== 2：丝杠高度 ====================
        draw_box(start_y + 2, start_x + 32, 8, 31, " ⚙️ 丝杠高度 ", 13);
        float abs_height = screw_pos_ + 662.0f;
        attron(A_BOLD | COLOR_PAIR(1));
        mvprintw(start_y + 4, start_x + 34, "📏 相对 Z:");
        attron(COLOR_PAIR(4)); printw("%6.1f mm", screw_pos_); attroff(COLOR_PAIR(4));
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
        int max_right_boundary = start_x + 95; 
        for (const auto& log : sys_logs_) {
            std::string prefix; int prefix_color;
            if (log.first >= rcl_interfaces::msg::Log::ERROR) {
                prefix = "❌"; prefix_color = 3; 
            } else if (log.first >= rcl_interfaces::msg::Log::WARN) {
                prefix = "⚠️"; prefix_color = 6; 
            } else {
                prefix = "💬"; prefix_color = 1;
            }
            print_colorized_log(log_y, start_x + 2, prefix, prefix_color, log.second, max_right_boundary);
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