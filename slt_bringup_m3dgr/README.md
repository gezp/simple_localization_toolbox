# slt_bringup_m3dgr

[M3DGR](https://github.com/sjtuyinjie/M3DGR) 数据集适配工具，包括两部分：数据回放、算法评测。

## 1. 数据回放

### 1.1 bag 格式转换

M3DGR 原始 bag 是 ROS1 格式，且激光为 livox CustomMsg，需要转换为 ROS2 mcap（PointCloud2）：

```bash
python3 scripts/rosbag1_to_rosbag2.py <input ROS1 .bag> <output_dir>
# 例：
python3 scripts/rosbag1_to_rosbag2.py \
  ~/localization_data/dataset/M3DGR/Standard/Outdoor01/Outdoor01.bag \
  ~/localization_data/dataset/M3DGR/Standard/Outdoor01/rosbag2
```

依赖：`pip install rosbags`

转换内容：

| 输入 topic                                                 | 输出 topic                        | 说明                                                                                 |
| ---------------------------------------------------------- | --------------------------------- | ------------------------------------------------------------------------------------ |
| `/livox/mid360/lidar`, `/livox/avia/lidar` (CustomMsg) | `/livox/*/points` (PointCloud2) | 字段 x y z intensity ring( u8) time(f64)，对齐 `slt_common::PointXYZIRT`，滤除零点 |
| `/livox/*/imu`, `/camera/imu`                          | 原名透传                          |                                                                                      |
| `/odom`                                                  | 原名透传                          | 轮速里程计                                                                           |
| `/ublox_driver/receiver_lla`                             | 原名透传                          | GNSS LLA                                                                             |
| 相机压缩图像                                               | 原名透传                          |                                                                                      |

跳过：`/mavros/*`（全程 NO_FIX）、gnss_comm 原始消息、CustomMsg 原始消息。

### 1.2 内外参文件格式

`config/calibration.yaml` 包含传感器外参与相机内参。

外参 `extrinsics` 为静态 TF 列表，每项含义：

```yaml
extrinsics:
  - frame_id: base_link        # 父坐标系
    child_frame_id: camera     # 子坐标系
    translation: [x, y, z]     # 平移，单位 m
    rotation: [r11, ..., r33]  # 3x3 旋转矩阵，行优先
```

即 `p_frame = R * p_child + t`。数值来源 M3DGR 官方标定，以
`base_link_T_avia = [0.6, 0, 0.15]`（单位旋转）为根，其余 `base_link_T_X` 链式推算。

TF 树：

```
base_link ─┬─ camera ── camera_imu
           ├─ livox_avia_lidar ── livox_avia_imu
           ├─ livox_mid360_lidar ── livox_mid360_imu
           └─ gnss_antenna
```

相机内参 `camera_intrinsics`：D435i RGB，pinhole 模型，含畸变参数 `k1 k2 p1 p2`
与投影参数 `fx fy cx cy`。

IMU 噪声 `imu_noise`：Allan 方差标定，`gyr_n/gyr_w/acc_n/acc_w`
（avia 与 mid360 内置 IMU 各一组）。

### 1.3 bag 回放 demo

```bash
ros2 launch slt_bringup_m3dgr bag_replay.launch.py \
  bag_path:=.../rosbag2 gt_path:=.../Outdoor01.txt rate:=1.0
```

- bag 播放 + rviz（点云、相机图像、TF）
- `replay_node` 发布静态外参 TF 与 ground truth `/m3dgr/ground_truth/odom`（frame: `map` → `base_link`）
- GT txt 格式：`timestamp x y z qx qy qz qw`，首帧为原点

M3DGR 的 GT 旋转均为单位阵，`replay_node` 支持从轨迹重建 yaw（`rebuild_gt_rotation`，默认开启），
pitch/roll 恒为 0。

## 2. 算法评测

### 2.1 lidar odometry

```bash
ros2 launch slt_bringup_m3dgr lidar_odometry.launch.py rate:=1.0
```

* 包含 bag 回放 + `replay_node` + `lidar_odometry_node`+ rviz。

评测方法：`simple_evaluator_node` 对比 `ground_truth` 与 `odom_lidar` 两条 odom 轨迹，
轨迹数据存 `~/localization_data/trajectory`。

### 2.2 TODO

后续计划支持：lidar mapping、lidar locator、ESKF locator、graph locator 等算法评测。
