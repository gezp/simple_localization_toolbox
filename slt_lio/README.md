# slt_lio

紧耦合LIO里程计，基于18维误差状态卡尔曼滤波(ESKF)与点到面残差。

运行demo

```bash
ros2 launch slt_lio lio_odometry.launch.py
```

* 可修改配置文件(`slt_lio/config/lio_odometry.yaml`)，支持两种不同的里程计方法：
  * `fastlio` : 关键帧局部地图，配准用pcl kdtree。
  * `fastlio2`：迭代重线性化，配准用`ikd-Tree`(见`third_party/ikd_tree`)。
* 发布两个里程计话题：
  * `lidar_odometry/odom` : 每个lidar观测一帧，带扫描匹配的修正结果。
  * `lidar_odometry/imu_odom` : imu频率的连续输出，观测到达时由修正结果重新播发。
* 启用`publish_tf`时，TF由`imu_odom`发布(不等待观测)。

保存轨迹

```bash
ros2 service call /save_odometry slt_interface/srv/SaveOdometry "{}"
```

* 保存的目录在`~/localization_data/trajectory/`

evo 轨迹评估

```bash
evo_ape tum ground_truth.txt odom_lidar.txt -a
evo_rpe tum ground_truth.txt odom_lidar.txt -a --delta 1 --delta_unit m
```
