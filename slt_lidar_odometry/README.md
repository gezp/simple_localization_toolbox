# slt_lidar_odometry

运行demo

```bash
ros2 launch slt_lidar_odometry lidar_odometry.launch.py
```

* 可修改配置文件(`slt_lidar_odometry/config/lidar_odometry.yaml`)，支持两种不同的里程计方法：
  * `simple` : 基于pcl中传统点云配准的方法，可以选择不同点云匹配方法（`ICP`, `ICP_SVD`, `NDT`, `NDT_OMP`）。
  * `loam`：基于`loam`点云特征的方法。

保存轨迹

```bash
# slt_lidar_odometry/scripts
bash save_odometry.sh
```

* 保存的目录在` ~/localization_data/trajectory/`

evo 轨迹评估

```bash
# slt_lidar_odometry/scripts
bash evo_ape.sh
bash evo_rpe.sh
```
