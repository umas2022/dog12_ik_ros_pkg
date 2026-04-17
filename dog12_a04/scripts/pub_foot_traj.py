'''
发布足端轨迹
'''

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

FOOT_NAMES = [
    "LF_FOOT",
    "RF_FOOT",
    "LH_FOOT",
    "RH_FOOT",
]
# Marker颜色分别，依次红、绿、蓝、黄
MARKER_COLORS = [
    (1.0, 0.0, 0.0),  # Red (LF)
    (0.0, 1.0, 0.0),  # Green (RF)
    (0.0, 0.0, 1.0),  # Blue (LH)
    (1.0, 1.0, 0.0)   # Yellow (RH)
]

class FootTrajPublisher(Node):
    def __init__(self):
        super().__init__('foot_traj_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 一组marker发布器
        self.marker_pubs = [self.create_publisher(Marker, f'/{name.lower()}_traj', 1) for name in FOOT_NAMES]
        self.pos_history = [[] for _ in FOOT_NAMES]  # 每个足端历史点
        self.max_len = 4000
        self.parent_frame = "base"   # 默认父坐标，需与你tf中一致，可改为base_link

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        for i, foot in enumerate(FOOT_NAMES):
            try:
                trans = self.tf_buffer.lookup_transform(self.parent_frame, foot, rclpy.time.Time())
                pos = trans.transform.translation
                pt = Point()
                pt.x, pt.y, pt.z = pos.x, pos.y, pos.z
                self.pos_history[i].append(pt)
                if len(self.pos_history[i]) > self.max_len:
                    self.pos_history[i].pop(0)
                # 构造marker
                marker = Marker()
                marker.header.frame_id = self.parent_frame
                marker.header.stamp = now
                marker.ns = foot + "_traj"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.01  # 线宽
                r, g, b = MARKER_COLORS[i]
                marker.color.a = 1.0
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.points = self.pos_history[i]
                self.marker_pubs[i].publish(marker)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                # 刚启动tf缓存空、未发布时允许弹出异常
                pass

def main():
    rclpy.init()
    node = FootTrajPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
