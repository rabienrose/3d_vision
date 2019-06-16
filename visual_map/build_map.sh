BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/zao_circle/20190522173131_qingtian_xiawu_2.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/lidar_vis_match
EXE_ROOT=/home/chamo/Documents/work/aimap/visual_map

BAG_EXTRACT_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe
VOC_ADDR=${EXE_ROOT}/devel/lib/vslam/create_vol
ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
SEG_ADDR=${EXE_ROOT}/devel/lib/seg_traj/seg_traj
LIDAR_GPS_ADDR=${EXE_ROOT}/devel/lib/convert_lidar_2_gps/convert_lidar_2_gps

#mkdir ${OUT_ADDR}/images
#${BAG_EXTRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} camera/left/image_raw imu/raw_data gps
#${LIDAR_GPS_ADDR} ${OUT_ADDR}

#${VOC_ADDR} ${OUT_ADDR}
${ORB_SLAM_ADDR} ${OUT_ADDR}/orbVoc.bin ${OUT_ADDR}/vslam.yaml ${BAG_NAME} ${OUT_ADDR} camera/left/image_raw
${SEG_ADDR} ${OUT_ADDR}
${OPTI_ADDR} ${OUT_ADDR}
#${INDEX_ADDR} ${OUT_ADDR}
