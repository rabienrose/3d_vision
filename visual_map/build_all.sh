catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build loc_lib
catkin build vslam
catkin build optimizer_tool
catkin build bag_tool
catkin build create_desc_index
catkin build seg_traj
catkin build convert_lidar_2_gps
catkin build convert_to_visual_map
