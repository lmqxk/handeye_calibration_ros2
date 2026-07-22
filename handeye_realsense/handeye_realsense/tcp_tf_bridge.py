"""
TCP → TF 桥接节点

订阅 /feedback/tcp_pose，发布 base_link → link6 TF 变换。
替代 robot_state_publisher，适用于 agx_arm_urdf 子模块未拉取的情况。

用法：
  ros2 run handeye_realsense tcp_tf_bridge

需要与 agx_arm_ctrl 同时运行。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np


class TcpTfBridge(Node):
    def __init__(self):
        super().__init__('tcp_tf_bridge')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.tcp_sub = self.create_subscription(
            PoseStamped, '/feedback/tcp_pose', self.tcp_callback, 10)

        self.get_logger().info("TCP→TF 桥接节点已启动，发布 base_link → link6")

    def tcp_callback(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'link6'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TcpTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
