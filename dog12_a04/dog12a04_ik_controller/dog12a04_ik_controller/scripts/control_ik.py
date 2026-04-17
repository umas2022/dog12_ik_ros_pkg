'''
x/y/yaw全向移动

实时修改方向：
ros2 param set /combined_motion_controller linear_x 1.0
ros2 param set /combined_motion_controller linear_y 0.0
ros2 param set /combined_motion_controller angular_z 0.0
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
import math

class CombinedMotionController(Node):
    JOINT_NAMES = [
    'LF_HAA', 'LF_HFE', 'LF_KFE',
    'RF_HAA', 'RF_HFE', 'RF_KFE',
    'LH_HAA', 'LH_HFE', 'LH_KFE',
    'RH_HAA', 'RH_HFE', 'RH_KFE'
    ]

    def __init__(self):
        super().__init__('combined_motion_controller')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # 机器人和步态参数
        self.l_thigh = 0.15
        self.l_shank = 0.10
        self.z_body = -0.18
        self.step_length_linear = 0.06
        self.step_length_angular = 0.06
        self.lift_height = 0.02
        self.period = 2.0
        self.Hz = 50
        self.dt = 1.0 / self.Hz

        self.init_angles = [0.0] * 12
        self.phases = {'LF': 0.0, 'RF': math.pi, 'LH': math.pi, 'RH': 0.0}
        self.leg_order = ['LF', 'RF', 'LH', 'RH']

        # ROS参数
        self.declare_parameter('linear_x', 0.0)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('angular_z', 0.0)

        # 关节符号
        self.joint_signs = {
            'LF': {'HAA': +1.0, 'HFE': +1.0, 'KFE': +1.0},
            'RF': {'HAA': -1.0, 'HFE': +1.0, 'KFE': +1.0},
            'LH': {'HAA': +1.0, 'HFE': -1.0, 'KFE': +1.0},
            'RH': {'HAA': -1.0, 'HFE': -1.0, 'KFE': +1.0},
        }
        
        self.leg_xy_signs = {
            'LF': np.array([+1.0, +1.0]), 'RF': np.array([+1.0, -1.0]),
            'LH': np.array([-1.0, +1.0]), 'RH': np.array([-1.0, -1.0]),
        }

        self.t = 0.0
        
        # ### <<< 关键修正 1：创建定时器 ###
        # 定时器会自动以 self.dt 的时间间隔调用 self.step 函数
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info('Combined Motion Controller started with Timer.')

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
        hfe, kfe_raw = CombinedMotionController.ik_leg_2d(x, z_prime, thigh, shank)
        return haa, hfe, kfe_raw

    @staticmethod
    def leg_foot_trajectory(phase, lift_height):
        if phase < math.pi:
            s_progress = -0.5 + (phase / math.pi)
            dz = 0.0
        else:
            s_progress = 0.5 - ((phase - math.pi) / math.pi)
            dz = lift_height * math.sin(phase - math.pi)
        return s_progress, dz

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
        # 1. 从ROS参数获取期望的线速度和角速度
        linear_x = self.get_parameter('linear_x').get_parameter_value().double_value
        linear_y = self.get_parameter('linear_y').get_parameter_value().double_value
        angular_z = self.get_parameter('angular_z').get_parameter_value().double_value

        linear_dir = np.array([linear_x, linear_y])
        linear_norm = np.linalg.norm(linear_dir)
        if linear_norm > 1.0:
            linear_dir /= linear_norm
        
        angular_z = np.clip(angular_z, -1.0, 1.0)

        motion_angles = []

        for leg in self.leg_order:
            phase = (2.0 * math.pi * (self.t / self.period) + self.phases[leg]) % (2.0 * math.pi)
            s_progress, dz = self.leg_foot_trajectory(phase, self.lift_height)
            
            # 平移位移
            foot_disp_linear = s_progress * self.step_length_linear * (-linear_dir)
            
            # 旋转位移
            yaw_dir = 1.0 if angular_z >=0 else -1.0
            xy_sign = self.leg_xy_signs[leg]
            tang_base = np.array([-xy_sign[1], xy_sign[0]])
            tang = yaw_dir * (tang_base / np.linalg.norm(tang_base))
            foot_disp_angular = s_progress * self.step_length_angular * abs(angular_z) * tang
            
            # 叠加
            final_foot_disp = foot_disp_linear + foot_disp_angular
            
            x_body = float(final_foot_disp[0])
            y_body = float(final_foot_disp[1])
            z_body = self.z_body + dz

            # IK和后续处理
            if leg in ["LF", "RF"]:
                x_for_ik = x_body
            else:
                x_for_ik = -x_body

            haa_geo, hfe_geo, kfe_raw_geo = self.ik_leg_3d_haa_xaxis(
                x_for_ik, y_body, z_body, self.l_thigh, self.l_shank
            )
            kfe_geo = math.pi - kfe_raw_geo
            haa_cmd, hfe_cmd, kfe_cmd = self.apply_joint_signs(leg, haa_geo, hfe_geo, kfe_geo)
            
            motion_angles.extend([haa_cmd, hfe_cmd, kfe_cmd])

        final_angles = [a + ia for a, ia in zip(motion_angles, self.init_angles)]
        self.send_angles(final_angles)
        self.t += self.dt

def main():
    rclpy.init()
    node = CombinedMotionController()
    try:
        # ### <<< 关键修正 2：使用 spin() ###
        # rclpy.spin() 会启动事件循环，处理定时器、服务请求、订阅消息等
        # 它会一直运行，直到节点被关闭 (例如通过 Ctrl+C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
