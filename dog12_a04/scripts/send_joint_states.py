'''
发布简单的关节状态消息到/joint_states话题。
'''


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleJointStatePublisher(Node):
    JOINT_NAMES = [
        'LF_HAA', 'LF_HFE', 'LF_KFE', 'RF_HAA', 'RF_HFE', 'RF_KFE',
        'LH_HAA', 'LH_HFE', 'LH_KFE', 'RH_HAA', 'RH_HFE', 'RH_KFE'
    ]
    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        
    def send_angles(self, angles):
        if len(angles) != len(self.JOINT_NAMES):
            self.get_logger().error(
                f"Expected {len(self.JOINT_NAMES)} angles, got {len(angles)}"
            )
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.JOINT_NAMES
        js.position = angles
        self.pub.publish(js)

def main():
    rclpy.init()
    node = SimpleJointStatePublisher()
    # 示例：让第一个关节随时间正弦运动，其它为0
    import math
    import time
    t = 0.0
    try:
        while rclpy.ok():
            angles = [math.sin(t)] + [0.0]*11
            node.send_angles(angles)
            t += 0.05
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
