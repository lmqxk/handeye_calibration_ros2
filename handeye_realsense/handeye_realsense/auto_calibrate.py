"""
自动手眼标定数据采集节点

从预设位姿列表逐个移动机械臂，等待到位后发布 'q' 到 keypress_topic，
触发 aruco_estimation + robot_state_piper 各自保存一组数据。

使用前提：
  1. 机械臂驱动 + CAN 已就绪（agx_arm_ctrl 节点运行中）
  2. aruco_estimation 节点已启动（显示 OpenCV 窗口，标定板在视野内）
  3. robot_state_piper 节点已启动（订阅 /feedback/tcp_pose）

用法：
  ros2 run handeye_realsense auto_calibrate

流程：
  逐个走到 18 个预设位姿 → 稳定 1.5s → 自动按 'q' 保存 → 走到下一位姿
  全部采集完成后自动回到起始位姿，然后节点退出。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.duration import Duration
from std_srvs.srv import SetBool
import numpy as np
import time


class AutoCalibrateNode(Node):
    def __init__(self):
        super().__init__('auto_calibrate_node')

        # 发布器
        self.joint_pub = self.create_publisher(JointState, '/control/move_j', 10)
        self.key_pub = self.create_publisher(String, 'keypress_topic', 10)

        # 服务客户端（使能机械臂）
        self.enable_cli = self.create_client(SetBool, '/enable_agx_arm')

        # 订阅器（用于判断到位）
        self.tcp_sub = self.create_subscription(
            PoseStamped, '/feedback/tcp_pose', self.tcp_callback, 10)

        # 位姿列表
        self.poses = self._generate_poses()
        self.total_poses = len(self.poses)
        self.current_pose_idx = -1  # 尚未开始
        self.completed = False      # 是否正常跑完全流程

        # TCP 位姿缓存 + 到位检测
        self.latest_pos = None
        self.prev_pos = None
        self.stable_count = 0
        self.STABLE_THRESHOLD = 0.005  # 米，位置变化 < 5mm 算稳定
        self.STABLE_FRAMES = 20        # 连续稳定 20 帧（~0.1s @ 200Hz）
        self.STABLE_WAIT = 4.0         # 到位后再额外等待 4s 让图像稳定（防抖动）

        self.timer = None              # 下一帧定时器
        self.phase = 'idle'            # idle / moving / settling / saving / done
        self.start_time = None
        self.settle_start_time = None

    def tcp_callback(self, msg: PoseStamped):
        self.latest_pos = msg.pose.position

    def _generate_poses(self) -> list:
        """基于起始位姿生成 18 组微小变化的关节角位姿（7 元素：6 关节 + 夹爪）"""
        # 起始姿态：已验证能使相机正对前方墙面
        # joint1=base, joint2=shoulder, joint3=elbow, joint4/5/6=wrist
        base = [0.0, 0.5, -0.3, 0.0, -0.2, 0.0]
        gripper = 0.08  # 夹爪开度（标定过程中夹爪保持打开）

        # 偏移量列表 [Δj0, Δj1, Δj2, Δj3, Δj4, Δj5]
        # 设计原则：小幅变化，确保 ArUco 始终在视野内
        offsets = [
            # ── 第 1 组：中心，不同距离 ──
            [0.00,  0.00,  0.00,  0.00,  0.00,  0.00],  # 1 初始
            [0.00,  0.08, -0.05,  0.00,  0.00,  0.00],  # 2 稍高靠近
            [0.00, -0.06,  0.05,  0.00,  0.00,  0.00],  # 3 稍低远离
            [0.00,  0.00, -0.08,  0.00,  0.00,  0.00],  # 4 伸更近
            [0.00,  0.00,  0.06,  0.00,  0.00,  0.00],  # 5 收更远

            # ── 第 2 组：左右平移 ──
            [0.15,  0.00, -0.05,  0.00,  0.00,  0.00],  # 6 右
            [-0.15,  0.00, -0.05,  0.00,  0.00,  0.00],  # 7 左
            [0.20,  0.05, -0.08,  0.00,  0.00,  0.00],  # 8 右+远
            [-0.20,  0.05, -0.08,  0.00,  0.00,  0.00],  # 9 左+远

            # ── 第 3 组：倾斜角度 ──
            [0.10,  0.05, -0.05,  0.15,  0.00,  0.00],  # 10 右+滚
            [-0.10,  0.05, -0.05, -0.15,  0.00,  0.00],  # 11 左+滚
            [0.00,  0.05, -0.05,  0.00,  0.10,  0.00],  # 12 俯仰
            [0.00,  0.00,  0.00,  0.00,  0.00,  0.20],  # 13 偏航

            # ── 第 4 组：组合极限 ──
            [0.12,  0.10, -0.10,  0.20,  0.05,  0.10],  # 14 右+高+滚+偏
            [-0.12,  0.10, -0.10, -0.20,  0.05, -0.10],  # 15 左+高+滚+偏
            [0.08, -0.05,  0.00,  0.00, -0.10,  0.15],  # 16 右+低+偏
            [-0.08, -0.05,  0.00,  0.00, -0.10, -0.15],  # 17 左+低+偏
            [0.00,  0.12, -0.12,  0.10,  0.10,  0.00],  # 18 最高+最近

            # ── 第 19：回到起始（全零偏移 = base）──
            [0.00,  0.00,  0.00,  0.00,  0.00,  0.00],  # 19 回到起始
        ]

        poses = []
        for off in offsets:
            joint_pos = [b + o for b, o in zip(base, off)]
            poses.append(joint_pos + [gripper])  # 加夹爪 = 7 元素
        return poses

    def start(self):
        """开始自动采集"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("🤖 自动手眼标定开始")
        self.get_logger().info(f"📋 共 {self.total_poses} 个位姿")
        self.get_logger().info("⚠️  请确保 aruco_estimation + robot_state_piper 已启动")
        self.get_logger().info("=" * 50)
        time.sleep(1)
        self._next_pose()

    def _next_pose(self):
        self.current_pose_idx += 1
        if self.current_pose_idx >= self.total_poses:
            self._finish()
            return

        pose = self.poses[self.current_pose_idx]
        idx = self.current_pose_idx + 1

        self.get_logger().info(f"\n[{idx}/{self.total_poses}] 移动到位姿...")

        # 发送关节指令
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        msg.position = pose  # 7 元素：6 关节 + 夹爪
        # 夹爪力：effort[6] = 1.0
        msg.effort = [0.0] * 7
        msg.effort[6] = 1.0
        self.joint_pub.publish(msg)

        # 进入到位检测阶段
        self.phase = 'moving'
        self.prev_pos = None
        self.stable_count = 0
        self.start_time = time.time()
        self._start_monitor()

    def _start_monitor(self):
        """启动到位监测定时器（100ms 间隔）"""
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(0.1, self._monitor_callback)

    def _monitor_callback(self):
        if self.phase == 'moving':
            self._check_arrival()
        elif self.phase == 'settling':
            self._check_settle()
        elif self.phase == 'saving':
            self._do_save()

    def _check_arrival(self):
        if self.latest_pos is None:
            return  # 还没收到反馈

        cur = np.array([self.latest_pos.x, self.latest_pos.y, self.latest_pos.z])

        if self.prev_pos is not None:
            delta = np.linalg.norm(cur - self.prev_pos)
            if delta < self.STABLE_THRESHOLD:
                self.stable_count += 1
            else:
                self.stable_count = 0

        self.prev_pos = cur

        # 超时保护（15s）
        elapsed = time.time() - self.start_time
        if elapsed > 15.0:
            self.get_logger().warn(f"⏱ 位姿 {self.current_pose_idx+1} 到位超时，强制继续")
            self._start_settle()
            return

        if self.stable_count >= self.STABLE_FRAMES:
            self._start_settle()

    def _start_settle(self):
        self.phase = 'settling'
        self.settle_start_time = time.time()
        self.get_logger().info(f"  已到位，等待 {self.STABLE_WAIT}s 让图像稳定...")

    def _check_settle(self):
        elapsed = time.time() - self.settle_start_time
        if elapsed >= self.STABLE_WAIT:
            self.phase = 'saving'
            self._do_save()

    def _do_save(self):
        idx = self.current_pose_idx + 1

        if idx < self.total_poses:
            # 前 18 个采集位姿：保存数据
            self.key_pub.publish(String(data='q'))
            self.get_logger().info(f"  ✅ [{idx}/{self.total_poses}] 已保存!")

            if self.latest_pos is not None:
                self.get_logger().info(
                    f"     TCP: x={self.latest_pos.x:.3f}, "
                    f"y={self.latest_pos.y:.3f}, "
                    f"z={self.latest_pos.z:.3f}")

            # 短暂等待 0.3s 确保文件写入完成
            time.sleep(0.3)

            # 进入下一位姿
            self.phase = 'idle'
            self._next_pose()
        else:
            # 第 19 个 = 回到起始位姿，已完成移动 → 结束
            self.get_logger().info("\n✅ 已回到起始位姿!")
            self._shutdown_node()

    def _enable_and_send_joints(self, positions, label="起始位姿"):
        """先使能机械臂，再发送关节指令"""
        # 调用使能服务
        self.get_logger().info(f"  使能机械臂...")
        req = SetBool.Request()
        req.data = True
        try:
            future = self.enable_cli.call(req, timeout_sec=5.0)
            if future.success:
                self.get_logger().info("  ✅ 使能成功")
            else:
                self.get_logger().warning(f"  ⚠️  使能返回失败: {future.message}")
        except Exception as e:
            self.get_logger().warn(f"  ⚠️  使能调用异常: {e}，继续发送关节指令...")

        time.sleep(0.3)

        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        msg.position = positions
        msg.effort = [0.0] * 7
        msg.effort[6] = 1.0
        self.joint_pub.publish(msg)
        self.get_logger().info(f"  已发送 {label} 关节指令")

    def _finish(self):
        self.phase = 'done'
        if self.timer is not None:
            self.timer.cancel()

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"   共采集 {self.total_poses - 1} 组数据")
        self.get_logger().info("   运行标定计算: ros2 run handeye_realsense handeye")
        self.get_logger().info("=" * 50)

    def _shutdown_node(self):
        """所有位姿走完，正常退出"""
        self.phase = 'done'
        self.completed = True
        if self.timer is not None:
            self.timer.cancel()
        self.get_logger().info("🛑 自动采集完成，节点退出")
        rclpy.shutdown()

    def on_shutdown(self):
        """节点退出时回到起始位姿（仅在异常退出时有效）"""
        if self.completed:
            return  # 正常走完全流程，已在起始位姿，无需重复移动
        self.get_logger().info("\n回到起始位姿...")
        base = [0.0, 0.5, -0.3, 0.0, -0.2, 0.0, 0.08]
        self._enable_and_send_joints(base, label="起始位姿")


def main(args=None):
    rclpy.init(args=args)
    node = AutoCalibrateNode()

    # 延迟 0.5s 后启动，等节点初始化
    start_timer = node.create_timer(0.5, lambda: (node.start(), start_timer.cancel()))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
