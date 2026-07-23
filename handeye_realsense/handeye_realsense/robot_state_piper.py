"""
AgileX Piper 机械臂 — 手眼标定末端姿态采集节点

订阅 /feedback/tcp_pose（末端在 base_link 中的位姿）,
按 'q' 键时保存当前 gripper2base 变换到 YAML 文件。

依赖 /feedback/tcp_pose（agx_arm_ctrl 驱动节点以 200Hz 发布）。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String

import yaml
import numpy as np
import os


class RobotStatePiperNode(Node):
    def __init__(self):
        super().__init__('robot_state_piper_node')

        # 相机选择参数：oak / realsense
        self.declare_parameter('camera', 'oak')
        camera = self.get_parameter('camera').value

        # 用脚本真实路径定位包目录
        self._pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        config_path = os.path.join(self._pkg_dir, f'config_{camera}.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.robot_data_file_name = os.path.join(self._pkg_dir, config["robot_data_file_name"])
        self.base_link = config["base_link"]
        self.ee_link = config["ee_link"]

        # 订阅 /feedback/tcp_pose（机械臂驱动发布的末端位姿）
        self.sub_tcp = self.create_subscription(
            PoseStamped, '/feedback/tcp_pose', self.tcp_callback, 10)
        self.sub_keypress = self.create_subscription(
            String, 'keypress_topic', self.keypress_callback, 10)

        self.pose_count = 0
        self.latest_pose = None  # 缓存最新位姿

        self.get_logger().info("robot_state_piper 已启动，等待 /feedback/tcp_pose...")
        self.get_logger().info("  按 'q' 保存当前位姿，'e' 退出")

        # 每 5s 打印一次状态，让用户知道节点还活着
        self.create_timer(5.0, self._status_timer)

    def _status_timer(self):
        if self.latest_pose is None:
            self.get_logger().info("等待 TCP 数据 — 请确认机械臂驱动已启动")
        else:
            self.get_logger().info("运行中，已保存 {} 组位姿".format(self.pose_count))

    def tcp_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def keypress_callback(self, msg: String):
        key = msg.data
        if key == 'q':
            if self.latest_pose is None:
                self.get_logger().warning("未收到 /feedback/tcp_pose，跳过此帧")
                return
            self.save_tcp_pose(self.latest_pose)
        elif key == 'e':
            self.get_logger().info("结束程序...")
            rclpy.shutdown()

    def save_tcp_pose(self, pose_msg: PoseStamped):
        """将 TCP 位姿（PoseStamped）转为旋转矩阵+平移向量，保存到 YAML"""
        p = pose_msg.pose.position
        o = pose_msg.pose.orientation

        # 四元数 → 旋转矩阵
        r = R.from_quat([o.x, o.y, o.z, o.w])
        R_mat = r.as_matrix()
        t_vec = np.array([p.x, p.y, p.z])

        # 加载已有数据
        try:
            with open(self.robot_data_file_name, 'r') as f:
                data = yaml.safe_load(f) or {'poses': []}
        except FileNotFoundError:
            data = {'poses': []}

        data['poses'].append({
            'rotation': R_mat.tolist(),
            'translation': t_vec.tolist()
        })

        with open(self.robot_data_file_name, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        self.pose_count += 1
        self.get_logger().info(f"=== Pose {self.pose_count} 已保存 ===")
        self.get_logger().info(f"    位置: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}")
        self.get_logger().info(f"    文件: {self.robot_data_file_name}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePiperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
