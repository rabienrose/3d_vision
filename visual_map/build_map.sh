BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/office_out_sunny/20190507121208_vision_l3.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/office_out_sunny
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

BAG_EXTRACT_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe
VOC_ADDR=${EXE_ROOT}/devel/lib/vslam/create_vol
ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test

mkdir ${OUT_ADDR}/images
${BAG_EXTRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} camera/right/image_raw imu/raw_data gps

${VOC_ADDR} ${OUT_ADDR}
${ORB_SLAM_ADDR} ${OUT_ADDR}/small_voc.yml ${OUT_ADDR}/vslam.yaml ${BAG_NAME} ${OUT_ADDR}
${OPTI_ADDR} ${OUT_ADDR}
${INDEX_ADDR} ${OUT_ADDR}
