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
        
        # 发布话题改为 C++ 节点监听的 /user_voice_cmd
        self.publisher_ = self.create_publisher(String, '/user_voice_cmd', 10)
        
        self.token = self.get_access_token()
        if not self.token:
            self.get_logger().error("🛑 无法获取百度 API Token！")
            return
            
        self.get_logger().info("✅ 百度大脑 Token 获取成功！")

        self.get_logger().info("==========================================")
        self.get_logger().info("🎙️ LuckRobot 语音监听端 (自适应防抖版) 已启动")
        
        # 自动扫描系统并锁定 USB 麦克风的真实动态序号
        self.MIC_INDEX = self.find_usb_mic_index()
        
        self.get_logger().info("==========================================")

        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 400 
        self.recognizer.dynamic_energy_threshold = True

        self.listen_thread = threading.Thread(target=self.audio_capture_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def find_usb_mic_index(self):
        """自动在系统中寻找 USB 麦克风，防止硬件序号漂移"""
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            if "USB Audio" in name or "Usb" in name:
                self.get_logger().info(f"💡 自动锁定 USB 麦克风: 序号 [{index}] -> 设备名: {name}")
                return index
        
        self.get_logger().warn("⚠️ 未找到带有 'USB' 字样的麦克风，将回退使用系统默认麦克风配置！")
        return None

    def get_access_token(self):
        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {"grant_type": "client_credentials", "client_id": API_KEY, "client_secret": SECRET_KEY}
        try:
            response = requests.post(url, params=params, timeout=5)
            return response.json().get("access_token")
        except Exception as e:
            return None

    def audio_capture_loop(self):
        try:
            # 🔥 核心修复：强制使用 48000Hz 硬件采样率和 2048 大缓存，专治 Jetson USB 麦克风兼容病
            with sr.Microphone(device_index=self.MIC_INDEX, sample_rate=48000, chunk_size=2048) as source:
                self.get_logger().info("🔧 正在校准环境噪音 (2秒)...")
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                self.get_logger().info("✅ 校准完成，我在听...")
                
                while rclpy.ok():
                    try:
                        audio = self.recognizer.listen(source, timeout=None, phrase_time_limit=5)
                        self.get_logger().info("⚡ 正在极速识别...")
                        
                        # 硬件是 48000Hz 录音的，在这里强行压缩回百度需要的 16000Hz
                        wav_data = audio.get_wav_data(convert_rate=16000, convert_width=2)
                        base64_audio = base64.b64encode(wav_data).decode('utf-8')
                        
                        payload = json.dumps({
                            "format": "wav", "rate": 16000, "channel": 1,
                            "cuid": "LuckRobot_VLA_001", "dev_pid": 80001,
                            "token": self.token, "speech": base64_audio, "len": len(wav_data)
                        }, ensure_ascii=False)

                        headers = {'Content-Type': 'application/json', 'Accept': 'application/json'}
                        response = requests.post(URL_RECOGNITION, headers=headers, data=payload.encode("utf-8"), timeout=5)
                        response.encoding = "utf-8"
                        result = response.json()

                        if result.get('err_no') == 0:
                            text = result['result'][0]
                            self.get_logger().info(f"🗣️ 听到指令: 「{text}」")
                            
                            # 直接将识别到的整句话原封不动发给 C++ 节点！
                            msg = String()
                            msg.data = text
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"📤 已将原声文字推送至 /user_voice_cmd 话题")
                            
                        else:
                            err_code = result.get('err_no', -1)
                            if err_code not in [3301, 3308]: 
                                self.get_logger().warn(f"❌ 识别失败，错误码: {err_code}")

                    except sr.WaitTimeoutError:
                        pass
                    except Exception as e:
                        self.get_logger().error(f"⚠️ 识别或网络异常: {e}")
                        time.sleep(1)

        except Exception as hardware_e:
            self.get_logger().error(f"🔥 麦克风硬件初始化彻底失败，请检查连线或采样率！详情: {hardware_e}")

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