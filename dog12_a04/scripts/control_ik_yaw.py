'''
z轴旋转
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
import math

class InPlaceYawJointStatePublisher(Node):
    JOINT_NAMES = [
    'LF_HAA', 'LF_HFE', 'LF_KFE',
    'RF_HAA', 'RF_HFE', 'RF_KFE',
    'LH_HAA', 'LH_HFE', 'LH_KFE',
    'RH_HAA', 'RH_HFE', 'RH_KFE'
    ]

    def __init__(self):
        super().__init__('in_place_yaw_joint_state_publisher')

        self.declare_parameter('topic', '/joint_states')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.pub = self.create_publisher(JointState, topic, 10)

        # 统一的物理参数
        self.l_thigh = 0.15
        self.l_shank = 0.10
        self.z_body = -0.18

        # 统一的步态参数
        self.step_length = 0.06
        self.lift_height = 0.06
        self.period = 2.0
        self.Hz = 50
        self.dt = 1.0 / self.Hz

        # 统一的初始角度
        self.init_angles = [0.0] * 12

        # 步态相位和顺序
        self.phases = {'LF': 0.0, 'RF': math.pi, 'LH': math.pi, 'RH': 0.0}
        self.leg_order = ['LF', 'RF', 'LH', 'RH']

        # ### <<< 修正点 1：恢复为符合您“外展为正”定义的正确关节符号
        self.joint_signs = {
            'LF': {'HAA': +1.0, 'HFE': +1.0, 'KFE': +1.0},
            'RF': {'HAA': -1.0, 'HFE': +1.0, 'KFE': +1.0},
            'LH': {'HAA': +1.0, 'HFE': -1.0, 'KFE': +1.0},
            'RH': {'HAA': -1.0, 'HFE': -1.0, 'KFE': +1.0},
        }

        # 1.0:顺时针，-1.0:逆时针
        self.declare_parameter('yaw_dir', -1.0)

        self.leg_xy_signs = {
            'LF': np.array([+1.0, +1.0]), 'RF': np.array([+1.0, -1.0]),
            'LH': np.array([-1.0, +1.0]), 'RH': np.array([-1.0, -1.0]),
        }

        self.t = 0.0
        self.get_logger().info(f'In-place yaw gait publisher started. Publishing to {topic}')

    def get_yaw_dir_from_params(self):
        v = self.get_parameter('yaw_dir').get_parameter_value().double_value
        return 1.0 if v >= 0 else -1.0

    @staticmethod
    def ik_leg_2d(x, z, thigh, shank):
        D = (x**2 + z**2 - thigh**2 - shank**2) / (2 * thigh * shank)
        D = np.clip(D, -1.0, 1.0)
        theta2_raw = math.atan2(math.sqrt(1 - D**2), -D)
        theta1 = math.atan2(x, -z) - math.atan2(shank * math.sin(theta2_raw), thigh - shank * math.cos(theta2_raw))
        return theta1, theta2_raw

    @staticmethod
    def ik_leg_3d_haa_xaxis(x, y, z, thigh, shank):
        haa = math.atan2(y, -z)
        z_prime = -math.sqrt(y**2 + z**2) if z < 0 else math.sqrt(y**2 + z**2)
        hfe, kfe_raw = InPlaceYawJointStatePublisher.ik_leg_2d(x, z_prime, thigh, shank)
        return haa, hfe, kfe_raw

    @staticmethod
    def leg_foot_trajectory(phase, step_length, lift_height):
        if phase < math.pi:
            s_tangential = -step_length / 2 + step_length * (phase / math.pi)
            dz = 0.0
        else:
            s_tangential = step_length / 2 - step_length * ((phase - math.pi) / math.pi)
            dz = lift_height * math.sin(phase - math.pi)
        return s_tangential, dz

    def apply_joint_signs(self, leg, haa, hfe, kfe):
        s = self.joint_signs[leg]
        return s['HAA'] * haa, s['HFE'] * hfe, s['KFE'] * kfe

    def send_angles(self, angles):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.JOINT_NAMES
        js.position = [float(a) for a in angles]
        self.pub.publish(js)

    def step(self):
        yaw_dir = self.get_yaw_dir_from_params()
        motion_angles = []

        for leg in self.leg_order:
            phase = (2.0 * math.pi * (self.t / self.period) + self.phases[leg]) % (2.0 * math.pi)
            
            s_tangential, dz = self.leg_foot_trajectory(phase, self.step_length, self.lift_height)
            xy_sign = self.leg_xy_signs[leg]
            tang_base = np.array([-xy_sign[1], xy_sign[0]])
            tang = yaw_dir * (tang_base / np.linalg.norm(tang_base))

            dxy = s_tangential * tang
            x_body = float(dxy[0])
            y_body = float(dxy[1])
            z_body = self.z_body + dz

            # ### <<< 修正点 2：翻转前后腿镜像逻辑，使其正确
            if leg in ["LF", "RF"]:
                # 前腿直接使用计算出的x坐标
                x_for_ik = x_body
            else: # ["LH", "RH"]
                # 后腿因为是镜像布局，需要使用反向的x坐标
                x_for_ik = -x_body

            # 2. 3D逆运动学解算
            haa_geo, hfe_geo, kfe_raw_geo = self.ik_leg_3d_haa_xaxis(
                x_for_ik, y_body, z_body, self.l_thigh, self.l_shank
            )
            
            # 3. KFE角度转换
            kfe_geo = math.pi - kfe_raw_geo

            # 4. 将几何角映射到URDF关节正方向
            haa_cmd, hfe_cmd, kfe_cmd = self.apply_joint_signs(leg, haa_geo, hfe_geo, kfe_geo)
            
            motion_angles.extend([haa_cmd, hfe_cmd, kfe_cmd])

        # 5. 叠加站立零姿态
        final_angles = [a + ia for a, ia in zip(motion_angles, self.init_angles)]
        self.send_angles(final_angles)
        self.t += self.dt

def main():
    rclpy.init()
    node = InPlaceYawJointStatePublisher()
    try:
        while rclpy.ok():
            node.step()
            time.sleep(node.dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
