BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/office_garage/20190422150408_.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/office_garage

IMG_TRACT_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/bag_tool/extract_img
IMU_TRACT_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/bag_tool/extract_imu
ORB_SLAM_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/vslam/mono_kitti
INDEX_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/optimizer_tool/optimizer_tool_test

#${IMG_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} camera/right/image_raw
#${IMU_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} imu/raw_data

#${IMG_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} img
#${IMU_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} imu

#${ORB_SLAM_ADDR} ${OUT_ADDR}/voc.yml ${OUT_ADDR}/vslam.yaml ${BAG_NAME} ${OUT_ADDR}

#${OPTI_ADDR} ${OUT_ADDR}

${INDEX_ADDR} ${OUT_ADDR}
