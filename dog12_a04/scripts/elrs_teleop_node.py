#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

# --- CRSF 协议常量和解析逻辑 (来自你的示例) ---
CRSF_ADDRESS = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
EXPECTED_PAYLOAD_LENGTH = 22

def parse_crsf_channels(payload):
    """从22字节的payload中解析16个通道数据"""
    if len(payload) != EXPECTED_PAYLOAD_LENGTH:
        return None

    # 将字节串转换为一个大整数
    bits = int.from_bytes(payload, byteorder="little")
    channels = []
    # 每个通道使用11位
    for i in range(16):
        value = (bits >> (i * 11)) & 0x7FF  # 0x7FF 是 2047，即11位的最大值
        channels.append(value)
    return channels


class ElrsTeleopNode(Node):
    def __init__(self):
        super().__init__('elrs_teleop_node')
        
        # --- ROS 2 参数定义 ---
        # 允许通过ros2 param set来修改这些值，而无需改代码
        self.declare_parameter('serial_port', '/dev/ttyUSB0') # 默认串口，Linux下常用名
        self.declare_parameter('baud_rate', 420000)
        self.declare_parameter('max_linear_x_vel', 1.0)  # 米/秒
        self.declare_parameter('max_linear_y_vel', 1.0)  # 米/秒
        self.declare_parameter('max_angular_z_vel', 1.0) # 弧度/秒

        # 获取参数值
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.max_linear_x = self.get_parameter('max_linear_x_vel').get_parameter_value().double_value
        self.max_linear_y = self.get_parameter('max_linear_y_vel').get_parameter_value().double_value
        self.max_angular_z = self.get_parameter('max_angular_z_vel').get_parameter_value().double_value

        # --- ROS 2 发布器 ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 串口和数据处理 ---
        self.ser = None
        self.buffer = bytearray()
        self.latest_channels = None
        self.running = True
        
        # --- 摇杆通道定义 (索引从0开始) ---
        # 根据你的描述:
        # yaw (angular.z) -> 通道 4 (索引 3) -> 摇杆左右
        # x   (linear.x)  -> 通道 2 (索引 1) -> 摇杆前后
        # y   (linear.y)  -> 通道 1 (索引 0) -> 摇杆左右
        self.chan_map = {
            'linear_x': 1,  # 通道2
            'linear_y': 0,  # 通道1
            'angular_z': 3, # 通道4
        }
        
        # --- 摇杆数值范围 ---
        self.min_val = 174
        self.mid_val = 992
        self.max_val = 1811

        # --- 在一个独立的线程中运行串口读取，避免阻塞ROS主循环 ---
        self.serial_thread = threading.Thread(target=self.serial_read_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # --- 创建一个定时器，定期发布Twist消息 ---
        self.timer = self.create_timer(0.1, self.publish_twist_message) # 10Hz

        self.get_logger().info(f"ELRS Teleop Node started. Reading from {self.serial_port} at {self.baud_rate} bps.")
        self.get_logger().info(f"Publishing to /cmd_vel.")

    def serial_read_loop(self):
        """串口读取和解析的循环"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
            self.running = False
            return

        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    self.buffer.extend(data)

                    # 尝试从 buffer 中提取合法的帧
                    while len(self.buffer) >= 3:
                        if self.buffer[0] != CRSF_ADDRESS:
                            self.buffer.pop(0)
                            continue

                        frame_len = self.buffer[1]
                        # 检查frame_len是否在合理范围内，避免因错误数据导致超大内存分配
                        if not (2 < frame_len < 64): 
                            self.buffer.pop(0)
                            continue

                        total_len = frame_len + 2
                        if len(self.buffer) < total_len:
                            break  # 等待更多数据

                        packet = self.buffer[:total_len]
                        self.buffer = self.buffer[total_len:]
                        
                        # 检查帧类型和长度是否匹配
                        if packet[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED and frame_len == EXPECTED_PAYLOAD_LENGTH + 2:
                            payload = packet[3:-1] # payload 在 type 和 crc之间
                            channels = parse_crsf_channels(payload)
                            if channels:
                                self.latest_channels = channels

            except serial.SerialException as e:
                self.get_logger().error(f"Serial port error: {e}")
                self.running = False
                break
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred in serial loop: {e}")
            time.sleep(0.005) # 短暂休眠，避免CPU空转

    def map_channel_to_velocity(self, value, is_inverted=False):
        """将单个通道的原始值 (174-1811) 映射到 -1.0 到 1.0"""
        # 添加一个小的死区
        if abs(value - self.mid_val) < 20:
            return 0.0

        if value > self.mid_val:
            normalized_val = (value - self.mid_val) / (self.max_val - self.mid_val)
        else:
            normalized_val = (value - self.mid_val) / (self.mid_val - self.min_val)
        
        # 根据你的描述，x轴是前大后小，所以需要反转
        if is_inverted:
            normalized_val *= -1
            
        return max(-1.0, min(1.0, normalized_val))

    def publish_twist_message(self):
        """根据最新的通道数据计算并发布Twist消息"""
        if self.latest_channels is None:
            return

        # 获取各轴的原始值
        raw_x = self.latest_channels[self.chan_map['linear_x']]
        raw_y = self.latest_channels[self.chan_map['linear_y']]
        raw_z = self.latest_channels[self.chan_map['angular_z']]

        # 映射到 -1.0 ~ 1.0
        # linear_x: 通道2, 前大后小, 无需反转
        norm_x = self.map_channel_to_velocity(raw_x, is_inverted=False) 
        # linear_y: 通道1, 左小右大, 需要反转
        norm_y = self.map_channel_to_velocity(raw_y,is_inverted=True) 
        # angular_z: 通道4, 左小右大, 无需反转
        norm_z = self.map_channel_to_velocity(raw_z)

        # 创建并填充Twist消息
        twist_msg = Twist()
        twist_msg.linear.x = norm_x * self.max_linear_x
        twist_msg.linear.y = norm_y * self.max_linear_y
        twist_msg.angular.z = norm_z * self.max_angular_z

        self.publisher_.publish(twist_msg)

    def destroy_node(self):
        """节点关闭时清理资源"""
        self.get_logger().info("Shutting down ELRS Teleop Node.")
        self.running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ElrsTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
