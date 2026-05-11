#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <libserial/SerialPort.h>
#include <cstdint>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

// 限位：0 ~ -462mm (最底端距离地面绝对高度 23cm)
const float MAX_LIMIT = 0.0f;
const float MIN_LIMIT = -462.0f;

const uint8_t SLAVE_ID = 1;
const std::string SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_ABALP8FF-if00-port0";
const LibSerial::BaudRate BAUD_RATE = LibSerial::BaudRate::BAUD_19200;
const LibSerial::Parity PARITY = LibSerial::Parity::PARITY_EVEN;
const LibSerial::StopBits STOP_BITS = LibSerial::StopBits::STOP_BITS_1;
const LibSerial::CharacterSize CHAR_SIZE = LibSerial::CharacterSize::CHAR_SIZE_8;

const float LEAD = 10.0f;
const int STEPS_PER_REV = 10000;
const int HOMING_SPEED = 20000;
const int MOVE_SPEED = 50000;

uint16_t modbus_crc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001; else crc >>= 1;
        }
    }
    return crc;
}

std::vector<uint8_t> build_write_single(uint8_t slave, uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> frame = {slave, 0x06, (uint8_t)(reg_addr>>8), (uint8_t)reg_addr, (uint8_t)(value>>8), (uint8_t)value};
    uint16_t crc = modbus_crc16(frame);
    frame.push_back(crc&0xFF); frame.push_back((crc>>8)&0xFF); return frame;
}

std::vector<uint8_t> build_write_multiple(uint8_t slave, uint16_t reg, uint16_t cnt, const std::vector<uint16_t>& vals) {
    std::vector<uint8_t> f = {slave, 0x10, (uint8_t)(reg>>8), (uint8_t)reg, (uint8_t)(cnt>>8), (uint8_t)cnt, (uint8_t)(cnt*2)};
    for (auto v : vals) { f.push_back(v>>8); f.push_back(v&0xFF); }
    uint16_t crc = modbus_crc16(f); f.push_back(crc&0xFF); f.push_back((crc>>8)&0xFF); return f;
}

class LeadScrewMotorNode : public rclcpp::Node {
public:
    LeadScrewMotorNode() : Node("lead_screw_motor_node"), serial_(std::make_shared<LibSerial::SerialPort>()) {
        if (!init_serial()) { RCLCPP_FATAL(this->get_logger(), "串口初始化失败"); rclcpp::shutdown(); return; }

        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lead_screw/displacement",
            rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile(),
            std::bind(&LeadScrewMotorNode::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<std_msgs::msg::Float32>("/lead_screw/current_position", 10);
        std::thread(&LeadScrewMotorNode::homing, this).detach();

        RCLCPP_INFO(this->get_logger(), "✅ 丝杠节点(绝对寻址+状态轮询版)");
        RCLCPP_INFO(this->get_logger(), "📏 限位：0(顶端) ~ -462mm(最底端,距地23cm)");
    }

private:
    std::mutex serial_mutex_;
    std::mutex move_mutex_;

    bool init_serial() {
        try {
            serial_->Open(SERIAL_PORT);
            serial_->SetBaudRate(BAUD_RATE); serial_->SetParity(PARITY);
            serial_->SetStopBits(STOP_BITS); serial_->SetCharacterSize(CHAR_SIZE);
            serial_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return true;
        } catch (...) { return false; }
    }

    void send(const std::vector<uint8_t>& f) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->Write(f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    bool send_and_receive(const std::vector<uint8_t>& tx_frame, std::vector<uint8_t>& rx_data, size_t expected_len) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->FlushIOBuffers(); 
        serial_->Write(tx_frame);
        
        rx_data.clear();
        auto start_time = std::chrono::steady_clock::now();
        while (rx_data.size() < expected_len) {
            if (serial_->IsDataAvailable()) {
                uint8_t byte;
                serial_->ReadByte(byte, 10);
                rx_data.push_back(byte);
            }
            if (std::chrono::steady_clock::now() - start_time > std::chrono::milliseconds(500)) {
                return false; 
            }
        }
        return true;
    }

    void homing() {
        RCLCPP_INFO(this->get_logger(), "🔧 触发回零中(寻找最高点 0)...");
        
        std::vector<std::vector<uint8_t>> cmds = {
            build_write_single(SLAVE_ID,0x0200,9), build_write_single(SLAVE_ID,0x1003,6),
            build_write_single(SLAVE_ID,0x1023,2), build_write_multiple(SLAVE_ID,0x1024,2,{(uint16_t)HOMING_SPEED,(uint16_t)(HOMING_SPEED>>16)}),
            build_write_single(SLAVE_ID,0x0307,0), build_write_single(SLAVE_ID,0x0D08,128)
        };
        for(auto c:cmds) send(c);

        std::vector<uint8_t> check_status_cmd = {0x01, 0x03, 0x0B, 0x07, 0x00, 0x02, 0x77, 0xEE};
        std::vector<uint8_t> target_reply = {0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x33};
        std::vector<uint8_t> rx_buf;

        RCLCPP_INFO(this->get_logger(), "🔍 正在高频查询电机内部回零状态...");

        bool is_homing_finished = false;
        while (rclcpp::ok() && !is_homing_finished) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
            
            if (send_and_receive(check_status_cmd, rx_buf, 9)) {
                if (rx_buf == target_reply) {
                    is_homing_finished = true;
                    RCLCPP_INFO(this->get_logger(), "🎯 收到确认回复！电机彻底回零完成！");
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "电机正在回零动作中...");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ 状态查询超时，总线可能繁忙，重试中...");
            }
        }

        RCLCPP_INFO(this->get_logger(), "⏹️ 发送解除回零状态指令...");
        send(build_write_single(SLAVE_ID, 0x0D08, 256));
        
        pos_ = 0.0f; 
        home_done_.store(true);
        publish_pos();
        RCLCPP_INFO(this->get_logger(), "✅ 丝杠节点已就绪，当前位置: 0.0 mm，允许接收绝对坐标指令！");
    }

    // 🔥 核心重构：参数变为 target_pos 绝对目标值
    void move(float target_pos) {
        if (!home_done_.load()) {
            RCLCPP_INFO(this->get_logger(), "⏳ 收到前往绝对高度 %.1f 的指令，但系统正在回零，已挂起...", target_pos);
            while (!home_done_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            RCLCPP_INFO(this->get_logger(), "▶️ 挂起结束！立即前往目标高度 %.1f", target_pos);
        }

        std::lock_guard<std::mutex> lock(move_mutex_);

        float curr = pos_.load();
        
        // 1. 安全限位校验
        if(target_pos > MAX_LIMIT || target_pos < MIN_LIMIT) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 超出限位保护：%.1f (允许范围: 0 ~ -462)", target_pos);
            return;
        }

        // 2. 计算本次需要的相对差值 (Delta)
        float delta = target_pos - curr;

        // 3. 极小位移防死锁逻辑（配合 Launch 文件）
        if (std::fabs(delta) < 0.1f) {
            RCLCPP_INFO(this->get_logger(), "ℹ️ 丝杠已在目标高度(%.1f mm)，无需重复动作。", target_pos);
            // ⚠️ 极其关键：即使不动，也必须打印这句话！
            // 因为你的 Launch 文件里的 grep 正在死等这句话！不打印会导致 Open3D 永远拉不起！
            RCLCPP_INFO(this->get_logger(), "📌 位置: %.1f mm", pos_.load());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "⚙️ 丝杠运行：当前 %.1f mm -> 目标 %.1f mm (产生位移差: %.1f mm)", curr, target_pos, delta);

        // 4. 将差值转为步数发给伺服电机
        float revs = delta / LEAD;
        int64_t steps = revs * STEPS_PER_REV;
        uint32_t s = (uint32_t)steps;

        std::vector<std::vector<uint8_t>> cmds = {
            build_write_single(SLAVE_ID,0x0200,9), build_write_single(SLAVE_ID,0x0D11,1), build_write_single(SLAVE_ID,0x1003,1),
            build_write_multiple(SLAVE_ID,0x100E,2,{(uint16_t)s,(uint16_t)(s>>16)}),
            build_write_multiple(SLAVE_ID,0x1019,2,{(uint16_t)MOVE_SPEED,(uint16_t)(MOVE_SPEED>>16)}),
            build_write_single(SLAVE_ID,0x0D12,507), build_write_single(SLAVE_ID,0x0D08,1)
        };
        for(auto c:cmds) send(c);

        pos_ = target_pos;
        publish_pos();

        // 5. 动态计算等待时间，等待物理动作完成
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(std::fabs(delta)*20)+500));
        send(build_write_single(SLAVE_ID,0x0D08,0));
        
        // 🔥 这一句将触发 Launch 脚本中的 wait_motor_finish_cmd！
        RCLCPP_INFO(this->get_logger(), "📌 位置: %.1f mm", pos_.load());
    }

    void callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // msg->data 现在代表绝对高度目标值
        std::thread(&LeadScrewMotorNode::move, this, msg->data).detach();
    }

    void publish_pos() {
        std_msgs::msg::Float32 m; m.data = pos_.load(); pub_->publish(m);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    std::shared_ptr<LibSerial::SerialPort> serial_;
    std::atomic<bool> home_done_{false};
    std::atomic<float> pos_{0.0f};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeadScrewMotorNode>());
    rclcpp::shutdown();
    return 0;
}