#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import sys
import math

sys.path.append("../..")  # 添加项目根目录到路径
from src.controller.ControllerDog12F import ControllerDog12F


class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("joint_state_subscriber")
        self.latest_joint_positions = [0.0] * 12  # 初始化12个关节位置
        self.subscription = self.create_subscription(JointState, "/joint_states", self.listener_callback, 10)

        # 定义目标关节顺序（与你的机器人控制器一致）
        self.target_joint_order = ["LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"]

    def listener_callback(self, msg):
        """
        更新最新的12个关节位置
        返回格式: [LF_HAA, LF_HFE, LF_KFE, RF_HAA,... RH_KFE]
        """
        temp_positions = [0.0] * 12  # 临时存储

        # 按照target_joint_order的顺序提取关节位置
        for i, joint_name in enumerate(self.target_joint_order):
            try:
                idx = msg.name.index(joint_name)
                temp_positions[i] = msg.position[idx]
            except ValueError:
                self.get_logger().warning(f"Joint {joint_name} not found in message")
                temp_positions[i] = 0.0  # 未找到关节时设为0

        self.latest_joint_positions = temp_positions

    def get_joint_positions(self):
        """获取最新的12个关节位置"""
        return self.latest_joint_positions


def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    test_agent = ControllerDog12F(mode="usb", serial_port="/dev/ttyCH343USB0")

    print("hardware init ...")
    test_agent.hardware_init()

    try:
        # 在线检查
        print("online check ...")
        while True:
            time.sleep(1)
            print("servo online check ...")
            if test_agent.online_check():
                print("All motors are online.")
                break

        # 初始化姿态
        print("position init ...")
        test_agent.move_all_init(1000, 50)
        time.sleep(2)
        test_agent.move_all_offset_from_stand([0] * 12, [1000] * 12, [50] * 12)
        time.sleep(2)

        while rclpy.ok():
            rclpy.spin_once(joint_state_subscriber)  # 处理一次回调

            # 获取当前关节位置
            joint_positions = joint_state_subscriber.get_joint_positions()

            joint_lf_haa_pos = -int(joint_positions[0] / math.pi * 2048)
            joint_lf_hfe_pos = int(joint_positions[1] / math.pi * 2048) + 1024
            joint_lf_kfe_pos = -int(joint_positions[2] / math.pi * 2048) + 2048

            joint_rf_haa_pos = -int(joint_positions[3] / math.pi * 2048)
            joint_rf_hfe_pos = int(joint_positions[4] / math.pi * 2048) + 1024
            joint_rf_kfe_pos = -int(joint_positions[5] / math.pi * 2048) + 2048

            joint_lh_haa_pos = int(joint_positions[6] / math.pi * 2048)
            joint_lh_hfe_pos = -int(joint_positions[7] / math.pi * 2048) + 1024
            joint_lh_kfe_pos = -int(joint_positions[8] / math.pi * 2048) + 2048

            joint_rh_haa_pos = int(joint_positions[9] / math.pi * 2048)
            joint_rh_hfe_pos = -int(joint_positions[10] / math.pi * 2048) + 1024
            joint_rh_kfe_pos = -int(joint_positions[11] / math.pi * 2048) + 2048

            joint_positions = [
                joint_lf_haa_pos,
                joint_lf_hfe_pos,
                joint_lf_kfe_pos,
                joint_rf_haa_pos,
                joint_rf_hfe_pos,
                joint_rf_kfe_pos,
                joint_lh_haa_pos,
                joint_lh_hfe_pos,
                joint_lh_kfe_pos,
                joint_rh_haa_pos,
                joint_rh_hfe_pos,
                joint_rh_kfe_pos,
            ]

            print(joint_positions)

            test_agent.move_all_offset(joint_positions, [1000] * 12, [50] * 12)

            time.sleep(0.02)  # 控制循环频率

    except KeyboardInterrupt:
        pass
    finally:
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()