'''
xy平面平移
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
import math

class OmniGaitJointStatePublisher(Node):
    JOINT_NAMES = [
    'LF_HAA', 'LF_HFE', 'LF_KFE',
    'RF_HAA', 'RF_HFE', 'RF_KFE',
    'LH_HAA', 'LH_HFE', 'LH_KFE',
    'RH_HAA', 'RH_HFE', 'RH_KFE'
    ]

    def __init__(self):
        super().__init__('omni_gait_joint_state_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Unified robot parameters
        self.l_thigh = 0.15
        self.l_shank = 0.10
        self.z_body = -0.18

        # Gait parameters
        self.step_length = 0.06
        self.lift_height = 0.06
        self.period = 2.0
        self.Hz = 50
        self.dt = 1.0 / self.Hz

        # Simplified initial angles
        self.init_angles = [0.0] * 12

        # Gait phasing and order
        self.phases = {'LF': 0.0, 'RF': math.pi, 'LH': math.pi, 'RH': 0.0}
        self.leg_order = ['LF', 'RF', 'LH', 'RH']

        # ROS parameters for walking direction
        # x>0向前,y>0向左
        self.declare_parameter('dir_x', 1.0)
        self.declare_parameter('dir_y', 1.0)

        # Correct joint signs for your robot
        self.joint_signs = {
            'LF': {'HAA': +1.0, 'HFE': +1.0, 'KFE': +1.0},
            'RF': {'HAA': -1.0, 'HFE': +1.0, 'KFE': +1.0},
            'LH': {'HAA': +1.0, 'HFE': -1.0, 'KFE': +1.0},
            'RH': {'HAA': -1.0, 'HFE': -1.0, 'KFE': +1.0},
        }

        self.t = 0.0
        self.get_logger().info('Omnidirectional gait publisher started.')

    def get_direction_from_params(self):
        dx = self.get_parameter('dir_x').get_parameter_value().double_value
        dy = self.get_parameter('dir_y').get_parameter_value().double_value
        v = np.array([dx, dy], dtype=float)
        n = np.linalg.norm(v)
        if n < 1e-6:
            return np.array([1.0, 0.0], dtype=float)
        else:
            return v / n

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
        hfe, kfe_raw = OmniGaitJointStatePublisher.ik_leg_2d(x, z_prime, thigh, shank)
        return haa, hfe, kfe_raw

    @staticmethod
    def leg_foot_trajectory(phase, step_length, lift_height):
        if phase < math.pi:
            s = -step_length / 2 + step_length * (phase / math.pi)
            dz = 0.0
        else:
            s = step_length / 2 - step_length * ((phase - math.pi) / math.pi)
            dz = lift_height * math.sin(phase - math.pi)
        return s, dz

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
        direction = self.get_direction_from_params()
        motion_angles = []

        for leg in self.leg_order:
            phase = (2.0 * math.pi * (self.t / self.period) + self.phases[leg]) % (2.0 * math.pi)
            s, dz = self.leg_foot_trajectory(phase, self.step_length, self.lift_height)

            # ### <<< THE CRITICAL FIX IS HERE ###
            # The foot must move in the OPPOSITE direction of the desired body motion
            # during the support phase to generate thrust.
            foot_displacement_vector = s * (-direction)

            x_body = float(foot_displacement_vector[0])
            y_body = float(foot_displacement_vector[1])
            z_body = self.z_body + dz

            # Apply correct Anymal-style IK input logic
            if leg in ["LF", "RF"]:
                x_for_ik = x_body
            else: # ["LH", "RH"]
                x_for_ik = -x_body

            # 3D IK
            haa_geo, hfe_geo, kfe_raw_geo = self.ik_leg_3d_haa_xaxis(
                x_for_ik, y_body, z_body, self.l_thigh, self.l_shank
            )
            
            # KFE angle conversion
            kfe_geo = math.pi - kfe_raw_geo

            # Apply joint signs
            haa_cmd, hfe_cmd, kfe_cmd = self.apply_joint_signs(leg, haa_geo, hfe_geo, kfe_geo)
            
            motion_angles.extend([haa_cmd, hfe_cmd, kfe_cmd])

        final_angles = [a + ia for a, ia in zip(motion_angles, self.init_angles)]
        self.send_angles(final_angles)
        self.t += self.dt

def main():
    rclpy.init()
    node = OmniGaitJointStatePublisher()
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
