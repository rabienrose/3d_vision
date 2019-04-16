BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/wayz_2019_04_08_huizhan/huizhanzhongxin.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/zhanhui

IMG_TRACT_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/bag_tool/extract_img
IMU_TRACT_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/bag_tool/extract_imu
ORB_SLAM_ADDR=/home/chamo/Documents/work/orb_mapping_loc
INDEX_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/test_orb_match/test_orb_match
OPTI_ADDR=/home/chamo/Documents/work/3d_vision/visual_map/devel/lib/optimizer_tool/optimizer_tool

#${IMG_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} camera/right/image_raw

#${IMU_TRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} imu/raw_data

${ORB_SLAM_ADDR}/Examples/Monocular/mono_kitti ${ORB_SLAM_ADDR}/Vocabulary/small_voc.yml ${OUT_ADDR}/orb_config.yaml ${OUT_ADDR}/camera_1_img ${OUT_ADDR} 1000 500000 false false

#${OPTI_ADDR} ${OUT_ADDR}

#${INDEX_ADDR} --images_folder=${OUT_ADDR}/camera_1_img --resource_dir=${OUT_ADDR}
