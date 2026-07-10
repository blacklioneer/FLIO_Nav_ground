#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr
import requests
import json
import base64
import threading
import time

# ☁️ 百度智能云 极速版 API 配置
API_KEY = "K5ltv0WHKPaTGlZha4CqTWhW"
SECRET_KEY = "mtt0mmTFbFqXOyjcwC4CSahSuGWQHtEg"
URL_RECOGNITION = "https://vop.baidu.com/pro_api"

class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__('voice_cmd_node')
        
        self.publisher_ = self.create_publisher(String, '/user_voice_cmd', 10)
        
        self.token = self.get_access_token()
        if not self.token:
            self.get_logger().error("🛑 无法获取百度 API Token！")
            return
            
        self.get_logger().info("✅ 百度大脑 Token 获取成功！")

        self.get_logger().info("==========================================")
        self.get_logger().info("🎙️ LuckRobot 语音监听端 (多线程不漏听版) 已启动")
        
        self.MIC_INDEX = self.find_usb_mic_index()
        
        self.get_logger().info("==========================================")

        self.recognizer = sr.Recognizer()
        
        # 🔥 优化 1：关闭动态阈值。机器人的底噪会让动态阈值越飘越高导致“耳聋”
        self.recognizer.dynamic_energy_threshold = False 
        # 🔥 优化 2：延长断句时间，允许用户说话时停顿 1.5 秒不断句（默认是0.8）
        self.recognizer.pause_threshold = 1.5
        # 🔥 优化 3：非语音环境下的静音限度，防止稍微一点杂音就录进去
        self.recognizer.non_speaking_duration = 0.5 

        self.listen_thread = threading.Thread(target=self.audio_capture_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def find_usb_mic_index(self):
        """自动在系统中寻找 USB 麦克风"""
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            if "USB Audio" in name or "Usb" in name:
                self.get_logger().info(f"💡 自动锁定 USB 麦克风: 序号 [{index}] -> 设备名: {name}")
                return index
        
        self.get_logger().warn("⚠️ 未找到带有 'USB' 字样的麦克风，将回退使用系统默认麦克风！")
        return None

    def get_access_token(self):
        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {"grant_type": "client_credentials", "client_id": API_KEY, "client_secret": SECRET_KEY}
        try:
            response = requests.post(url, params=params, timeout=5)
            return response.json().get("access_token")
        except Exception as e:
            return None

    # 🔥 优化 4：将网络请求拆分到独立函数，以便放进单独的线程里跑
    def process_audio_thread(self, audio):
        try:
            # 硬件 48000Hz 压缩回百度需要的 16000Hz
            wav_data = audio.get_wav_data(convert_rate=16000, convert_width=2)
            base64_audio = base64.b64encode(wav_data).decode('utf-8')
            
            payload = json.dumps({
                "format": "wav", "rate": 16000, "channel": 1,
                "cuid": "LuckRobot_VLA_001", "dev_pid": 80001,
                "token": self.token, "speech": base64_audio, "len": len(wav_data)
            }, ensure_ascii=False)

            headers = {'Content-Type': 'application/json', 'Accept': 'application/json'}
            # self.get_logger().info("☁️ 正在上传云端解析...")
            response = requests.post(URL_RECOGNITION, headers=headers, data=payload.encode("utf-8"), timeout=5)
            response.encoding = "utf-8"
            result = response.json()

            if result.get('err_no') == 0:
                text = result['result'][0]
                self.get_logger().info(f"🗣️ 听到指令: 「{text}」")
                
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                # self.get_logger().info(f"📤 已推送至 /user_voice_cmd")
                
            else:
                err_code = result.get('err_no', -1)
                # 3301是静音，3308是太长，忽略即可
                if err_code not in [3301, 3308]: 
                    self.get_logger().warn(f"❌ 识别失败，错误码: {err_code}")

        except Exception as e:
            self.get_logger().error(f"⚠️ 云端处理异常: {e}")

    def audio_capture_loop(self):
        try:
            with sr.Microphone(device_index=self.MIC_INDEX, sample_rate=48000, chunk_size=2048) as source:
                self.get_logger().info("🔧 正在采集环境底噪，请保持安静 (2秒)...")
                # 收集2秒环境噪音，并自动锁定此刻的能量阈值，以后不再乱飘
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                
                # 稍微再提高一点点静态门槛，防止被底盘轻微震动误触发
                self.recognizer.energy_threshold += 150 
                
                self.get_logger().info(f"✅ 底噪校准完成 (锁定阈值: {int(self.recognizer.energy_threshold)})，我在听...")
                
                while rclpy.ok():
                    try:
                        # phrase_time_limit 改为 8 秒，给你更充分的表达时间
                        audio = self.recognizer.listen(source, timeout=None, phrase_time_limit=8)
                        
                        # 🔥 最核心优化：录音一结束，立刻丢进新的线程去请求百度！
                        # 这样主循环能瞬间回到上一行 listen() 继续听你说话，绝对不漏音！
                        threading.Thread(target=self.process_audio_thread, args=(audio,), daemon=True).start()
                        
                    except sr.WaitTimeoutError:
                        pass
                    except Exception as e:
                        self.get_logger().error(f"⚠️ 拾音异常: {e}")
                        time.sleep(1)

        except Exception as hardware_e:
            self.get_logger().error(f"🔥 麦克风硬件初始化彻底失败！详情: {hardware_e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()