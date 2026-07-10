#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp> // 🔥 修正：ROS 2 中 UInt8 的头文件叫 u_int8.hpp
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <mutex>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <thread>

const char* SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
const int BAUDRATE = B115200;          
const uint8_t LEFT_DANGER_CMD[] = {0xAA, 0x55, 0x00, 0x0A, 0xFB};  
const uint8_t RIGHT_DANGER_CMD[] = {0xAA, 0x55, 0x00, 0x0B, 0xFB}; 

// 系统启动与音量配置指令
const uint8_t VOL_MAX_CMD[] = {0xAA, 0x55, 0x06, 0x00, 0xFB}; // 音量最大
const uint8_t STARTUP_CMD[] = {0xAA, 0x55, 0x00, 0x0C, 0xFB}; // 播报：启动完毕

const int CMD_LENGTH = 5;              
const int COOLDOWN_MS = 3000;
const int SERIAL_WRITE_TIMEOUT_MS = 500; 
const int MAX_INIT_RETRIES = 15;       
const int RETRY_WAIT_MS = 2000;        

class DangerCommandNode : public rclcpp::Node {
public:
    DangerCommandNode() : Node("danger_command_node"), serial_fd_(-1),
                          left_dangerous_(false), right_dangerous_(false),
                          last_left_send_ms_(0), last_right_send_ms_(0), retry_count_(0) {
        
        RCLCPP_INFO(this->get_logger(), "🚀 危险报警及语音播报节点已启动 (异步保活版)");

        obstacle_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "obstacle_distances", 10, std::bind(&DangerCommandNode::obstacle_callback, this, std::placeholders::_1));

        // 专门接收语音播报控制码的话题
        voice_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/voice_prompt", 10, std::bind(&DangerCommandNode::voice_callback, this, std::placeholders::_1));

        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&DangerCommandNode::check_and_send, this));

        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(RETRY_WAIT_MS), std::bind(&DangerCommandNode::async_init_serial, this));

        async_init_serial();
    }

    ~DangerCommandNode() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_fd_ >= 0) close(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "=== 报警节点安全退出 ===");
    }

private:
    void async_init_serial() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_fd_ >= 0) return;

        if (init_serial()) {
            RCLCPP_INFO(this->get_logger(), "✅ 报警器串口连接成功！");
            if (init_timer_) init_timer_->cancel();
            retry_count_ = 0;

            // 串口一旦连上，立刻发送音量最大指令
            tcflush(serial_fd_, TCOFLUSH);
            write(serial_fd_, VOL_MAX_CMD, CMD_LENGTH);
            tcdrain(serial_fd_);
            
            // 🔥 删除了开机播报指令，把它交给宏观的 Launch 文件在最后时刻去调度！
            RCLCPP_INFO(this->get_logger(), "📢 扬声器音量已调至最大，静候调度指令...");

        } else {
            retry_count_++;
            if (retry_count_ >= MAX_INIT_RETRIES) {
                RCLCPP_FATAL(this->get_logger(), "❌ 报警器连接彻底失败，节点终止！");
                rclcpp::shutdown();
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000, 
                                     "⏳ 报警器未连接，后台重连中 (%d/%d)...", retry_count_, MAX_INIT_RETRIES);
            }
        }
    }

    bool init_serial() {
        serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        fcntl(serial_fd_, F_SETFL, 0);
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd_, &tty) != 0) {
            close(serial_fd_); serial_fd_ = -1; return false;
        }

        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |= CS8 | CREAD | CLOCAL;
        cfsetispeed(&tty, BAUDRATE); cfsetospeed(&tty, BAUDRATE);
        tty.c_cc[VTIME] = SERIAL_WRITE_TIMEOUT_MS / 100;
        tty.c_cc[VMIN] = CMD_LENGTH;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            close(serial_fd_); serial_fd_ = -1; return false;
        }
        tcflush(serial_fd_, TCIOFLUSH);
        return true;
    }

    bool send_command(const uint8_t* cmd, int cmd_len) {
        std::lock_guard<std::mutex> lock(serial_mutex_); 
        if (serial_fd_ < 0 || !cmd) return false;

        tcflush(serial_fd_, TCOFLUSH);
        ssize_t sent = write(serial_fd_, cmd, cmd_len);
        
        if (sent != cmd_len) {
            RCLCPP_ERROR(this->get_logger(), "⚠️ 发送失败，报警器USB可能已断开！触发重新连接...");
            close(serial_fd_);
            serial_fd_ = -1;
            if (init_timer_) init_timer_->reset();
            return false;
        }
        tcdrain(serial_fd_);
        return true;
    }

    // 专门接收任务节点的通用播报指令
    void voice_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        uint8_t cmd_code = msg->data;
        // 动态拼装指令帧：AA 55 00 [控制码] FB
        uint8_t voice_frame[] = {0xAA, 0x55, 0x00, cmd_code, 0xFB};
        
        if (send_command(voice_frame, CMD_LENGTH)) {
            RCLCPP_INFO(this->get_logger(), "📢 成功触发任务语音播报: 0x%02X", cmd_code);
        }
    }

    void obstacle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (msg->data.size() >= 4) {
            left_dangerous_ = (msg->data[2] == 1.0);
            right_dangerous_ = (msg->data[3] == 1.0);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "⚠️ 收到不合法的数据长度: 期望4，实际%ld", msg->data.size());
        }
    }

    void check_and_send() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (serial_fd_ < 0) return;

        uint64_t now_ms = this->now().nanoseconds() / 1000000;

        if (left_dangerous_) {
            if ((last_left_send_ms_ == 0 ? COOLDOWN_MS + 1 : now_ms - last_left_send_ms_) >= COOLDOWN_MS) {
                if (send_command(LEFT_DANGER_CMD, CMD_LENGTH)) {
                    last_left_send_ms_ = now_ms;
                    RCLCPP_WARN(this->get_logger(), "🚨 左侧触发避障！已发送声光报警指令");
                }
            }
        }
        if (right_dangerous_) {
            if ((last_right_send_ms_ == 0 ? COOLDOWN_MS + 1 : now_ms - last_right_send_ms_) >= COOLDOWN_MS) {
                if (send_command(RIGHT_DANGER_CMD, CMD_LENGTH)) {
                    last_right_send_ms_ = now_ms;
                    RCLCPP_WARN(this->get_logger(), "🚨 右侧触发避障！已发送声光报警指令");
                }
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr voice_sub_; // 语音播报订阅
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::TimerBase::SharedPtr init_timer_; 
    
    int serial_fd_;                         
    std::mutex serial_mutex_; 
    bool left_dangerous_, right_dangerous_;                   
    uint64_t last_left_send_ms_, last_right_send_ms_;           
    int retry_count_;
    std::mutex state_mutex_;                
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DangerCommandNode>());
    rclcpp::shutdown();
    return 0;
}