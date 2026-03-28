data_dir=~/localization_data
evo_rpe tum ${data_dir}/trajectory/ground_truth.txt ${data_dir}/trajectory/lidar_pose.txt -r trans_part --delta 100 --plot --plot_mode xyz