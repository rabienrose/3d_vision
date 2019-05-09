BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_office/05-05-12-07-12.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_office/v3_try

BAG_EXTRACT_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/bag_tool/bag_tool_exe
ORB_SLAM_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/vslam/mono_kitti
INDEX_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/optimizer_tool/optimizer_tool_test

#${IMG_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} camera/right/image_raw
#${IMU_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} imu/raw_data

${BAG_EXTRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} img imu gps

#${ORB_SLAM_ADDR} ${OUT_ADDR}/orbVoc.bin ${OUT_ADDR}/vslam.yaml ${BAG_NAME} ${OUT_ADDR}

#${OPTI_ADDR} ${OUT_ADDR}

#${INDEX_ADDR} ${OUT_ADDR}
