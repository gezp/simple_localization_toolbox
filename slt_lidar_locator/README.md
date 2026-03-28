# lidar_localizarion

运行demo

```bash
ros2 launch slt_lidar_locator matching.launch.py 
```

* 可修改配置文件(`slt_lidar_locator/config/lidar_locator.yaml`)，选择不同点云匹配方法：`ICP`, `ICP_SVD`, `NDT`, `NDT_OMP`
* 运行之前需要先进行建图，确保` ~/localization_data/map/` 下已经有地图文件。

保存轨迹

```bash
# slt_lidar_locator/scripts
bash save_odometry.sh
```

* 保存的目录在` ~/localization_data/trajectory/`

evo 轨迹评估

```bash
# slt_lidar_locator/scripts
bash evo_ape.sh
bash evo_rpe.sh
```
