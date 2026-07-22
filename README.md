# handeye_calibration_ros2 — Piper + OAK-D 适配

> 基于 [shengyangzhuang/handeye_calibration_ros2](https://github.com/shengyangzhuang/handeye_calibration_ros2) 的 fork，适配 **AgileX PiPER 6-DOF 机械臂 + OAK-D 相机**（eye-in-hand）。
>
> 原项目作者：Zhuang, Shengyang（Imperial College London）
> 论文：*Multi-Robot System Prototyping for Cooperative Control in Robot-Assisted Spine Surgery*

---

## 改动内容

| 类型 | 文件 | 说明 |
|------|------|------|
| **新增** | `robot_state_piper.py` | 订阅 `/feedback/tcp_pose` 采集末端位姿（替代原版 TF 链式拼接） |
| **新增** | `auto_calibrate.py` | 自动走到 18 个预设位姿，到位后自动保存，无需手动按键 |
| **新增** | `tcp_tf_bridge.py` | 发布 `base_link → link6` TF（从 TCP 实时反馈读取） |
| **新增** | `scripts/convert_camera_info.py` | OAK-D camera_info → OpenCV FileStorage 格式转换 |
| **新增** | `oak_info.yaml` | OAK-D 相机内参（OpenCV 格式） |
| **修改** | `config.yaml` | 适配 OAK-D topic、frame_id、ArUco 字典 |
| **修改** | `aruco_estimation.py` | OAK-D 图像格式适配 |
| **修改** | `handeye_estimation.py` | 标定计算兼容优化 |
| **修改** | `publish_eye2hand.py` | frame_id 适配 Piper |
| **修改** | `setup.py` | 注册新节点入口 |

## 标定结果

使用 18 组位姿 + Tsai-Lenz 算法：

```
旋转矩阵 R_cam2gripper (link6 → camera):
  [[ 0.1079,  0.9404,  0.3224],
   [-0.9942,  0.1025,  0.0337],
   [-0.0013, -0.3242,  0.9460]]

平移向量 t_cam2gripper (m):
  [-0.0979, -0.0057,  0.0325]

四元数 (xyzw):
  [-0.1219,  0.1102, -0.6587,  0.7342]
```

### TF 验证

```
ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_optical_frame
- Translation: [0.107, 0.008, 0.265]
```

## 硬件环境

| 组件 | 型号 |
|------|------|
| 机械臂 | AgileX PiPER 6-DOF + 夹爪 |
| 相机 | OAK-D（eye-in-hand 安装） |
| 标定板 | ArUco 6×6, ID 365, 150mm |
| 主控 | Unitree Go2 NX 核心板 |
| OS | Ubuntu 22.04, ROS2 Humble |

## 安装

```bash
mkdir -p ~/handeye_ws/src
cd ~/handeye_ws/src
git clone https://github.com/lmqxk/handeye_calibration_ros2.git
cd ~/handeye_ws
source /opt/ros/humble/setup.bash
pip install transforms3d scipy
colcon build --symlink-install
source install/setup.bash
```

## 快速使用

### 自动采集（推荐）

```bash
# 终端 1: 机械臂驱动
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    arm_type:=piper auto_enable:=true effector_type:=agx_gripper

# 终端 2: OAK-D 相机
ros2 launch depthai_ros_driver camera.launch.py

# 终端 3: 标定节点（两个标签页）
source ~/handeye_ws/install/setup.bash
ros2 run handeye_realsense robot_piper      # 标签页 A
ros2 run handeye_realsense aruco            # 标签页 B

# 终端 4: 自动采集
ros2 run handeye_realsense auto_calibrate
```

### 计算标定结果

```bash
ros2 run handeye_realsense handeye
```

### 发布 TF

```bash
# base_link → link6（从 /feedback/tcp_pose 读取）
ros2 run handeye_realsense tcp_tf_bridge &

# link6 → camera（标定结果）
ros2 run handeye_realsense eye2hand
```

## 许可证

Apache-2.0（同原项目）

---

> 原项目地址：[https://github.com/shengyangzhuang/handeye_calibration_ros2](https://github.com/shengyangzhuang/handeye_calibration_ros2)
