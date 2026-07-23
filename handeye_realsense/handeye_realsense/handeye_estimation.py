"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
import os


class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration_node')
        self.get_logger().info("Starting Hand-Eye Calibration Node")

        # 相机选择参数：oak / realsense
        self.declare_parameter('camera', 'oak')
        camera = self.get_parameter('camera').value

        self._pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        config_path = os.path.join(self._pkg_dir, f'config_{camera}.yaml')
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        self.robot_data_file_name = os.path.join(self._pkg_dir, config["robot_data_file_name"])
        self.marker_data_file_name = os.path.join(self._pkg_dir, config["marker_data_file_name"])
        self.handeye_result_file_name = os.path.join(self._pkg_dir, config["handeye_result_file_name"])
        self.handeye_result_profile_file_name = os.path.join(self._pkg_dir, config["handeye_result_profile_file_name"])

        # 延迟一帧执行，确保节点初始化完成后再计算
        self._calc_timer = self.create_timer(0.1, self._compute_and_exit)

    def _compute_and_exit(self):
        self._calc_timer.cancel()

        # Load transformation data from YAML files
        self.R_gripper2base, self.t_gripper2base = self.load_transformations(self.robot_data_file_name)
        self.R_target2cam, self.t_target2cam = self.load_transformations(self.marker_data_file_name)

        robot_count = len(self.R_gripper2base)
        marker_count = len(self.R_target2cam)
        self.get_logger().info(f"机器人数据: {robot_count} 组")
        self.get_logger().info(f"标定板数据: {marker_count} 组")

        if robot_count != marker_count:
            self.get_logger().warn(
                f"两组数据数量不一致 ({robot_count} vs {marker_count})，"
                f"自动取前 {min(robot_count, marker_count)} 组对齐")
            # 按顺序取最小数量（数据按采集顺序保存，前 N 组对应）
            n = min(robot_count, marker_count)
            self.R_gripper2base = self.R_gripper2base[:n]
            self.t_gripper2base = self.t_gripper2base[:n]
            self.R_target2cam = self.R_target2cam[:n]
            self.t_target2cam = self.t_target2cam[:n]

        pairs = len(self.R_gripper2base)
        self.get_logger().info(f"有效标定组数: {pairs}")

        if pairs < 5:
            self.get_logger().error(
                f"有效数据仅 {pairs} 组，严重不足！需要至少 5 组。请重新采集")
            rclpy.shutdown()
            return
        elif pairs < 8:
            self.get_logger().warn(
                f"有效数据 {pairs} 组，偏少（建议 ≥ 8 组），可以尝试计算但结果可能不稳定")
        else:
            self.get_logger().info(
                f"有效数据 {pairs} 组，足够计算 ✅")

        # 检查是否有旧数据混合的迹象
        if robot_count > pairs * 1.5 or marker_count > pairs * 1.5:
            self.get_logger().warn(
                "原始数据量远大于有效组数，可能有旧数据残留在文件中。"
                "如需清理，删除对应 resource/*.yaml 数据文件后重新采集")

        # Compute the hand-eye transformation matrix
        self.compute_hand_eye()

        # 计算完成，退出节点
        rclpy.shutdown()

    def load_transformations(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            poses = data['poses']

        # Initialize to handle yaml data format
        R = []
        t = []

        for pose in poses:
            rotation = np.array(pose['rotation'], dtype=np.float32)
            translation = np.array(pose['translation'], dtype=np.float32)

            R.append(rotation)
            t.append(translation)

        return R, t

    def compute_hand_eye(self):
        self.get_logger().info(f"Loaded {len(self.R_gripper2base)} rotations and {len(self.t_gripper2base)} translations for gripper to base")
        self.get_logger().info(f"Loaded {len(self.R_target2cam)} rotations and {len(self.t_target2cam)} translations for target to camera")
        rotations = [r.reshape(3, 3) for r in self.R_gripper2base]
        translations = [t.reshape(3, 1) for t in self.t_gripper2base]
        obj_rotations = [r.reshape(3, 3) for r in self.R_target2cam]
        obj_translations = [t.reshape(3, 1) for t in self.t_target2cam]


        # Perform hand-eye calibration
        R, t = cv2.calibrateHandEye(
            rotations, translations, obj_rotations, obj_translations,
            method=cv2.CALIB_HAND_EYE_TSAI)

        # Save results to YAML
        # Output: camera relative to gripper frame (eye to hand)
        self.save_yaml(R, t)
        #self.save_yaml_profile(R_qua, t)

    def rotation_matrix_to_quaternion(self, matrix):
        """Convert a 3x3 rotation matrix into a quaternion."""
        rotation = R.from_matrix(matrix)
        return rotation.as_quat()

    def save_yaml(self, R, t):
        '''This function will always show only the updated result'''
        new_data = {'rotation': R.flatten().tolist(), 'translation': t.flatten().tolist()}

        # Write the new data to the YAML file, overwriting any existing content
        with open(self.handeye_result_file_name, 'w') as file:
            yaml.safe_dump(new_data, file)

        self.get_logger().info("Hand-eye calibration results saved.")
        print(f"Rotation matrix: {R}")
        print(f"Translation vector: {t}")

    def save_yaml_profile(self, R, t):
        '''This function saves the rotation and translation data in the correct format.'''
        new_data = {'rotation': R.flatten().tolist(), 'translation': t.flatten().tolist()}

        # Check if the file exists and is not empty
        if os.path.exists(self.handeye_result_profile_file_name) and os.path.getsize(self.handeye_result_profile_file_name) > 0:
            # Load the existing data from the file
            with open(self.handeye_result_profile_file_name, 'r') as file:
                existing_data = yaml.safe_load(file)

            # If the file contains data, append the new transform
            if 'transforms' in existing_data:
                existing_data['transforms'].append(new_data)
            else:
                existing_data = {'transforms': [new_data]}
        else:
            # If the file does not exist or is empty, start with a new structure
            existing_data = {'transforms': [new_data]}

        # Save the updated structure back to the file
        with open(self.handeye_result_profile_file_name, 'w') as file:
            yaml.safe_dump(existing_data, file)

        self.get_logger().info("Hand-eye calibration results saved.")
        print(f"Rotation matrix quaternion: {R}")
        print(f"Translation vector: {t}")


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
