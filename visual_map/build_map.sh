BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/acuu_test/20190722112619_jt_2.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/acuu_test/222
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

BAG_EXTRACT_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe
ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti
CONV_MAP_ADDR=${EXE_ROOT}/devel/lib/convert_to_visual_map/convert_to_visual_map
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
SEG_ADDR=${EXE_ROOT}/devel/lib/seg_traj/seg_traj
DSO_ADDR=${EXE_ROOT}/devel/lib/dso/dso_test 
POSE_EXTRACTOR=${EXE_ROOT}/devel/lib/global_match_test/global_match_test 
LIDAR_GPS_ADDR=${EXE_ROOT}/devel/lib/convert_lidar_2_gps/convert_lidar_2_gps

mkdir ${OUT_ADDR}/images
${BAG_EXTRACT_ADDR} --bag_addr=${BAG_NAME} --out_dir=${OUT_ADDR} --imu_topic=imu/raw_data --img_topic=camera/left/image_raw --gps_topic=navsatfix --isExtractImage=false --gps_anchor_x=31.1829017405 --gps_anchor_y=121.601053003667 --gps_anchor_z=15.792
#${LIDAR_GPS_ADDR} ${OUT_ADDR} -2862007.58909608 4651578.76768603 3283278.9620274

#${ORB_SLAM_ADDR} --bag_addr=${BAG_NAME} --output_addr=${OUT_ADDR}/ --voc_addr=${OUT_ADDR}/FreakAll.bin --camera_config=${OUT_ADDR}/camera_config.txt --image_topic=camera/left/image_raw --min_frame=0 --max_frame=10000 --step_frame=1 --use_orb=false --feature_count=2000 --feature_scale_factor=1.2 --feature_level=8 --min_match_count=100 --max_step_KF=15 --v=0 --logtostderr=true --colorlogtostderr=true
#${CONV_MAP_ADDR} --res_root=${OUT_ADDR} --map_name=chamo.map visualmap
#${SEG_ADDR} --map_addr=${OUT_ADDR} --map_name=chamo.map --err_thres=10.5
#${OPTI_ADDR} --map_addr=${OUT_ADDR} --map_name=1000_chamo.map --opti_type=BA --opti_count=100 --gps_weight=1000000 --t_c_g_x=0.1 --t_c_g_y=0.0 --t_c_g_z=0.0
#${INDEX_ADDR} ${OUT_ADDR} opti_1000_chamo.map
