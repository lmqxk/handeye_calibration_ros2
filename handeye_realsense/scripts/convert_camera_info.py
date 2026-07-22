#!/usr/bin/env python3
"""
ROS CameraInfo YAML → OpenCV FileStorage YAML 转换工具

用法：
    python3 convert_camera_info.py <camera_info.yaml> [-o output.yaml]

示例：
    python3 convert_camera_info.py ~/agx_arm_ws/data/photos/20260722_201436_740471/camera_info.yaml

输出 OpenCV 格式的 YAML 文件，包含 K 和 D 矩阵，
供 handeye_calibration_ros2 的 aruco_estimation.py 加载内参。
"""
import argparse
import yaml
import numpy as np

def ros_camera_info_to_opencv(input_path, output_path=None):
    """读取 ROS CameraInfo YAML，输出 OpenCV FileStorage YAML"""
    with open(input_path, 'r') as f:
        data = yaml.safe_load(f)

    # camera_info.yaml 有 color/depth 两个部分，取 color
    cam = data.get('color', data)

    # 提取 K 矩阵（3x3）
    k_values = cam.get('k') or cam.get('K', [])
    # 提取 D 向量（畸变系数）
    d_values = cam.get('d') or cam.get('D', [])

    if not k_values or len(k_values) < 9:
        print(f"错误：未找到有效的 K 矩阵（从 {input_path}）")
        return

    # OpenCV rational_polynomial 有 8 个系数
    # 如果全零，截断到 5 个（plumb_bob）即可
    # 保留非零系数
    d_len = len(d_values)
    if d_len > 8 and all(abs(v) < 1e-10 for v in d_values):
        d_values = d_values[:5]  # 全零截断到 5

    # OpenCV FileStorage 格式
    opencv_yaml = f"""%YAML:1.0
---
K: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [{', '.join(f'{v:.16f}' for v in k_values)}]
D: !!opencv-matrix
   rows: 1
   cols: {len(d_values)}
   dt: d
   data: [{', '.join(f'{v:.8f}' for v in d_values)}]
"""

    out_path = output_path or 'opencv_camera_info.yaml'
    with open(out_path, 'w') as f:
        f.write(opencv_yaml)

    print(f"✓ 已转换: {input_path} → {out_path}")
    print(f"  K: {k_values[0]:.3f}  {k_values[1]:.3f}  {k_values[2]:.3f}")
    print(f"     {k_values[3]:.3f}  {k_values[4]:.3f}  {k_values[5]:.3f}")
    print(f"     {k_values[6]:.3f}  {k_values[7]:.3f}  {k_values[8]:.3f}")
    print(f"  D: {len(d_values)} 个系数, 首项={d_values[0] if d_values else 0}")


def main():
    parser = argparse.ArgumentParser(description='ROS CameraInfo YAML → OpenCV FileStorage YAML')
    parser.add_argument('input', help='ROS camera_info.yaml 文件路径')
    parser.add_argument('-o', '--output', help='输出文件路径（默认 opencv_camera_info.yaml）')
    args = parser.parse_args()
    ros_camera_info_to_opencv(args.input, args.output)


if __name__ == '__main__':
    main()
