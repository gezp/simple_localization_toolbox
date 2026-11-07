# slt_vio

单目视觉惯性里程计，基于滑动窗口的视觉惯性紧耦合优化，算法与
[VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) 一致，不含 pose_graph 与在线外参标定。

运行demo

```bash
ros2 launch slt_bringup_m3dgr vio.launch.py
# 指定数据
ros2 launch slt_bringup_m3dgr vio.launch.py bag_path:=<rosbag2 目录> gt_path:=<gt.txt> rate:=0.5
```

* 需要 `slt_bringup_m3dgr` 回放数据集，相机内参、外参由 `replay_node` 从标定文件发布。
* 可修改配置文件(`slt_vio/config/vio_odometry.yaml`)，跟踪、初始化、求解器各读自己的 section。
* 发布三个话题：
  * `visual_odometry/odom` : 观测优化后的位姿。
  * `visual_odometry/imu_odom` : imu 频率的连续输出，观测到达时由修正结果重新播发。
  * `visual_odometry/feature_image` : 跟踪结果可视化。

运行手机单目demo

```bash
# 终端1: 播放rosbag (需要包含 /camera/image_raw/compressed 与 /imu/data)
ros2 bag play <phone_bag>
# 终端2: 启动标定管理、vio与rviz
ros2 launch slt_vio phone_vio.launch.py
```

* 相机内参、外参、时间偏移由 `calib_manager_node` 从 `slt_vio/resource/phone_calibration.yaml` 发布，vio 通过 tf 与 `camera_info` 获取。
* 配置文件为 `slt_vio/config/phone_vio.yaml`，与 `vio_odometry.yaml` 结构相同，针对手机 8Hz 图像调整了窗口长度与噪声权重。

保存轨迹

```bash
bash slt_vio/scripts/save_odometry.sh
```

* 保存的目录在`~/localization_data/trajectory_vio/`

evo 轨迹评估

```bash
evo_ape tum ground_truth.txt odom_vio.txt -a
evo_rpe tum ground_truth.txt odom_vio.txt -a --delta 1 --delta_unit m
```
