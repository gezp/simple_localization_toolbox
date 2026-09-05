# slt_bringup_m3dgr

[M3DGR](https://github.com/sjtuyinjie/M3DGR) 数据集 ROS2 回放工具（bag 转换 + 回放 + lidar odometry 评测）。

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
发布全部静态外参 TF（`base_link`、`livox_mid360_lidar` 等）。

## 3. lidar odometry 评测

```bash
ros2 launch slt_bringup_m3dgr lidar_odometry.launch.py
```

包含：bag 回放 + `replay_node`（静态外参 TF + GT 回放）+ `lidar_odometry_node`
（simple + NDT，开启去畸变，单位阵起步，通过 TF 获取 `base_link` → `livox_mid360_lidar` 外参）

+ `simple_evaluator_node`（ground_truth vs lidar_odom，轨迹存 `~/localization_data/trajectory`）+ rviz。
