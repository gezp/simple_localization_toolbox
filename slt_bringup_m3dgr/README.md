# slt_bringup_m3dgr

[M3DGR](https://github.com/sjtuyinjie/M3DGR) 数据集适配工具：数据集格式转换（ROS1 → ROS2）+ 算法 bringup（回放、lidar odometry、评测）。

## 1. 转换 bag

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

## 2. 回放

```bash
# bag + ground truth 回放 + rviz
ros2 launch slt_bringup_m3dgr bag_replay.launch.py \
  bag_path:=.../rosbag2 gt_path:=.../Outdoor01.txt rate:=1.0
```

ground truth txt（`timestamp x y z qx qy qz qw`，首帧为原点）由 `replay_node` 发布为
`/m3dgr/ground_truth/odom`（frame: `map` → `base_link`），同时按 `config/calibration.yaml`
发布全部静态外参 TF（`base_link`、`livox_avia_lidar`、`livox_mid360_lidar`、`camera` 等）。

M3DGR 的 GT 旋转均为单位阵，`replay_node` 支持从轨迹重建 yaw（`rebuild_gt_rotation`，默认开启）：
以 ±1 m 弧长窗口的弦方向为目标航向，弦长不足（静止/漂移噪声）时保持上一帧，相邻运动帧间
限幅 ±5°，pitch/roll 恒为 0。

## 3. lidar odometry 评测

```bash
ros2 launch slt_bringup_m3dgr lidar_odometry.launch.py rate:=4.0
```

包含：

- bag 回放 + `replay_node`（静态外参 TF + GT 回放，含 GT yaw 重建）
- `lidar_odometry_node`（simple + NDT，avia 点云 `/livox/avia/points`，
  单位阵起步，通过 TF 获取 `base_link` → `livox_avia_lidar` 外参）
- `simple_evaluator_node`（ground_truth vs lidar_odom，轨迹存 `~/localization_data/trajectory`）
- rviz

注意：`undistort_point_cloud` 默认关闭——它使用上一帧 odom twist，avia 非重复扫描
（100 ms 积分）在转弯时角速度突变，滞后 twist 会抹花点云导致 NDT 发散（z/pitch 飞掉）。
Outdoor01 实测：关闭后全程稳定，x 方向轨迹与 GT 贴合。

## 4. TF 树

```
map ── odom_lidar ── base_link ─┬─ camera ── camera_imu
                               ├─ livox_avia_lidar ── livox_avia_imu
                               ├─ livox_mid360_lidar ── livox_mid360_imu
                               └─ gnss_antenna
```

`config/calibration.yaml` 外参来源 M3DGR 官方标定，以 `base_link_T_avia = [0.6, 0, 0.15]`
（单位旋转）为根，其余 `base_link_T_X` 链式推算。
