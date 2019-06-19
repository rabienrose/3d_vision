#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/bag_match_lidar/06-12-14-09-52.bag
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/garage_3.bag
BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/office_1.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/bag_match_lidar/test_720_30_1
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

BAG_EXTRACT_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe
VOC_ADDR=${EXE_ROOT}/devel/lib/vslam/create_vol
ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
SEG_ADDR=${EXE_ROOT}/devel/lib/seg_traj/seg_traj

#mkdir ${OUT_ADDR}/images
#${BAG_EXTRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} img imu gps

${VOC_ADDR} ${OUT_ADDR}/ORBTest.bin "orb" 1000 5 3 ${OUT_ADDR}/images_building ${OUT_ADDR}/images_tree
#${ORB_SLAM_ADDR} ${OUT_ADDR}/FreakOffice.bin ${OUT_ADDR}/vslam.yaml ${BAG_NAME} ${OUT_ADDR} img
#${SEG_ADDR} ${OUT_ADDR}
#${OPTI_ADDR} ${OUT_ADDR} chamo_1000.map
#${INDEX_ADDR} ${OUT_ADDR} chamo_1000.map
