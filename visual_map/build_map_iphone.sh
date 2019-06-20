OUT_ADDR=$1
BAG_NAME=$2

echo working directory ${OUT_ADDR}
echo bag address ${BAG_NAME}


#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/office_2.bag
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/garage_3.bag
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/office_1.bag
#OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/bag_match_lidar/test_720_30_2
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

BAG_EXTRACT_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe
ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti
CONV_MAP_ADDR=${EXE_ROOT}/devel/lib/convert_to_visual_map/convert_to_visual_map
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
SEG_ADDR=${EXE_ROOT}/devel/lib/seg_traj/seg_traj

mkdir ${OUT_ADDR}/images
${BAG_EXTRACT_ADDR} ${BAG_NAME} ${OUT_ADDR} img imu gps

${ORB_SLAM_ADDR} ${OUT_ADDR}/FreakAll.bin ${OUT_ADDR}/vslam.yaml ${BAG_NAME} ${OUT_ADDR} img 0 100000 1
${CONV_MAP_ADDR} ${OUT_ADDR}
${SEG_ADDR} ${OUT_ADDR}
${OPTI_ADDR} ${OUT_ADDR} chamo_1000.map
${INDEX_ADDR} ${OUT_ADDR} opti_chamo_1000.map
