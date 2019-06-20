BAG_NAME1=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/garage_1.bag
BAG_NAME2=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/office_garage_1.bag
BAG_NAME3=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/garage_2.bag
BAG_NAME4=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/garage_3.bag
BAG_NAME5=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/building_near_1.bag
BAG_NAME6=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/building_up.bag
BAG_NAME7=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/office_1.bag
BAG_NAME8=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/building_near.bag
BAG_NAME9=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/building_up.bag
BAG_NAME10=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/office_2.bag
BAG_NAME11=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/building_near.bag
BAG_NAME12=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/building_up.bag
BAG_NAME13=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/office.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/voc_gen_ws
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

BAG_EXTRACT_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe
VOC_ADDR=${EXE_ROOT}/devel/lib/vslam/create_vol

mkdir ${OUT_ADDR}/1
mkdir ${OUT_ADDR}/1/images
mkdir ${OUT_ADDR}/2
mkdir ${OUT_ADDR}/2/images
mkdir ${OUT_ADDR}/3
mkdir ${OUT_ADDR}/3/images
mkdir ${OUT_ADDR}/4
mkdir ${OUT_ADDR}/4/images
mkdir ${OUT_ADDR}/5
mkdir ${OUT_ADDR}/5/images
mkdir ${OUT_ADDR}/6
mkdir ${OUT_ADDR}/6/images
mkdir ${OUT_ADDR}/7
mkdir ${OUT_ADDR}/7/images
mkdir ${OUT_ADDR}/8
mkdir ${OUT_ADDR}/8/images
mkdir ${OUT_ADDR}/9
mkdir ${OUT_ADDR}/9/images
mkdir ${OUT_ADDR}/10
mkdir ${OUT_ADDR}/10/images
mkdir ${OUT_ADDR}/11
mkdir ${OUT_ADDR}/11/images
mkdir ${OUT_ADDR}/12
mkdir ${OUT_ADDR}/12/images
mkdir ${OUT_ADDR}/13
mkdir ${OUT_ADDR}/13/images
${BAG_EXTRACT_ADDR} ${BAG_NAME1} ${OUT_ADDR}/1 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME2} ${OUT_ADDR}/2 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME3} ${OUT_ADDR}/3 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME1} ${OUT_ADDR}/4 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME2} ${OUT_ADDR}/5 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME3} ${OUT_ADDR}/6 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME1} ${OUT_ADDR}/7 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME2} ${OUT_ADDR}/8 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME3} ${OUT_ADDR}/9 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME1} ${OUT_ADDR}/10 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME2} ${OUT_ADDR}/11 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME3} ${OUT_ADDR}/12 img imu gps
${BAG_EXTRACT_ADDR} ${BAG_NAME3} ${OUT_ADDR}/13 img imu gps

${VOC_ADDR} ${OUT_ADDR}/FREAKTest.bin "freak" 3000 10 6 ${OUT_ADDR}/1/images ${OUT_ADDR}/2/images ${OUT_ADDR}/3/images ${OUT_ADDR}/4/images ${OUT_ADDR}/5/images ${OUT_ADDR}/6/images ${OUT_ADDR}/7/images ${OUT_ADDR}/8/images ${OUT_ADDR}/9/images ${OUT_ADDR}/10/images ${OUT_ADDR}/11/images ${OUT_ADDR}/12/images ${OUT_ADDR}/13/images
