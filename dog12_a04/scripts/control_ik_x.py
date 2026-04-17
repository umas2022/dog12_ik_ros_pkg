'''
x轴直线前进、后退
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
import math

class SimpleJointStatePublisher(Node):
    JOINT_NAMES = [
    'LF_HAA', 'LF_HFE', 'LF_KFE',
    'RF_HAA', 'RF_HFE', 'RF_KFE',
    'LH_HAA', 'LH_HFE', 'LH_KFE',
    'RH_HAA', 'RH_HFE', 'RH_KFE'
    ]

    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # 机器人参数
        self.l_thigh = 0.15  # thigh length
        self.l_shank = 0.10  # shank length
        self.z_body = -0.18 
        self.step_length = 0.06
        self.lift_height = 0.06
        self.period = 2.0
        self.Hz = 50
        self.dt = 1.0 / self.Hz

        # 初始姿态角度（URDF关节空间，单位：弧度）
        self.init_angles = [
            0/180*math.pi, 0/180*math.pi, 0/180*math.pi,  # LF
            0/180*math.pi, 0/180*math.pi, 0/180*math.pi,  # RF
            0/180*math.pi, 0/180*math.pi, 0/180*math.pi,  # LH
            0/180*math.pi, 0/180*math.pi, 0/180*math.pi,  # RH
        ]

        # 腿部相位（对角步态）
        self.phases = {'LF': 0.0, 'RF': math.pi, 'LH': math.pi, 'RH': 0.0}
        self.leg_order = ['LF', 'RF', 'LH', 'RH']

        # 每条腿各关节轴的正负号映射：几何IK角 -> URDF关节正方向
        self.joint_signs = {
            'LF': {'HAA': +1.0, 'HFE': +1.0, 'KFE': +1.0},
            'RF': {'HAA': -1.0, 'HFE': +1.0, 'KFE': +1.0},
            'LH': {'HAA': +1.0, 'HFE': -1.0, 'KFE': +1.0},
            'RH': {'HAA': -1.0, 'HFE': -1.0, 'KFE': +1.0},
        }

        self.t = 0.0

    @staticmethod
    def ik_leg(x, z, thigh, shank):
        # 2自由度平面机械臂逆运动学（几何角）
        # 肩关节O，大小腿关节A，足端P(坐标(x,z))，大腿与-z轴夹角theta1（逆时针正），大小腿夹角theta2；OA大腿长度记thigh，AP小腿长度记shank，OP=sqrt(x^2+y^2)
        # 余弦定理：c^2 = a^2 + b^2 -2*a*b*cos(C),记 D = -cos(theta2)可得下式
        D = (x**2 + z**2 - thigh**2 - shank**2) / (2 * thigh * shank)
        # 将 D 的取值限定在 [-1.0, 1.0] 之间
        D = np.clip(D, -1.0, 1.0)
        # [-pi,pi]内arccos(-D)会有正负两个解， 用atan2替代可以区分象限，theta = atan2(sin(theta),cos(theta)),取正方向
        theta2 = math.atan2(math.sqrt(1 - D**2), -D)
        # 大腿转角theta1 = 总方向（足端夹角POZ） - 偏移补偿（大腿足端夹角AOP）
        # atan2函数中两个参数并不一定要严格按照sin/cos关系归一化，只要能表示向量方向即可
        # 小腿AP作沿大腿方向和垂直大腿方向的投影，分别是shank*cos(theta2)和shank*sin(theta2)，在大腿方向上投影垂足为Q，在三角形OPQ中可以解角QOP即AOP
        theta1 = math.atan2(x, -z) - math.atan2(shank * math.sin(theta2), thigh - shank * math.cos(theta2))
        
        return theta1, theta2

    @staticmethod
    def leg_foot_trajectory(phase, step_length, lift_height):
        # 支撑相线性、摆动相抬脚（sin 顶）
        if phase < math.pi:
            x = -step_length / 2 + step_length * (phase / math.pi)  # support
            z = 0.0
        else:
            x = step_length / 2 - step_length * ((phase - math.pi) / math.pi)  # swing
            z = lift_height * math.sin((phase - math.pi) / math.pi * math.pi)
        return x, z

    def apply_joint_signs(self, leg, haa, hfe, kfe):
        s = self.joint_signs[leg]
        return s['HAA'] * haa, s['HFE'] * hfe, s['KFE'] * kfe

    def send_angles(self, angles):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.JOINT_NAMES
        js.position = [float(a) for a in angles]
        self.pub.publish(js)

    def send_all_zero(self):
        zeros = [0.0] * len(self.JOINT_NAMES)
        self.send_angles(zeros)

    def send_all_init(self):
        self.send_angles(self.init_angles)

    def send_all_from_init(self,angle):
        self.send_angles([angle]*12)

    def send_all_from_zero(self,angle):
        angle_list = [x+angle for x in self.init_angles]
        self.send_angles(angle_list)


    def step(self):
        angles = []
        for leg in self.leg_order:
            phase = (2.0 * math.pi * (self.t / self.period) + self.phases[leg]) % (2.0 * math.pi)
            x, dz = self.leg_foot_trajectory(phase, self.step_length, self.lift_height)
            z = self.z_body + dz

            # HAA 在此简化为 0（如需侧移/转向再开启）
            # 前后腿镜像布局，LH和RH的HFE关节方向相反，x取反
            haa_geo = 0.0
            if leg in ["LF","RF"]:
                hfe_geo, kfe_geo = self.ik_leg(-x, z,self.l_thigh,self.l_shank)
            elif leg in ["LH","RH"]:
                hfe_geo, kfe_geo = self.ik_leg(x, z,self.l_thigh,self.l_shank)
            kfe_geo = math.pi-kfe_geo # 折算为电机转角

            # 将几何角映射到URDF关节正方向
            haa_cmd, hfe_cmd, kfe_cmd = self.apply_joint_signs(leg, haa_geo, hfe_geo, kfe_geo)

            angles.extend([haa_cmd, hfe_cmd, kfe_cmd])

        # 叠加站立零姿态（URDF关节空间）
        final_angles = [a + ia for a, ia in zip(angles, self.init_angles)]
        self.send_angles(final_angles)

        self.t += self.dt

def main():
    rclpy.init()
    node = SimpleJointStatePublisher()
    try:
        while rclpy.ok():
            # node.send_all_init()
            # node.send_all_zero()
            # node.send_all_from_zero(0/180*math.pi)
            # node.send_all_from_init(0/180*math.pi)
            node.step()
            time.sleep(node.dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()