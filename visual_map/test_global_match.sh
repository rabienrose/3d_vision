#OUT_ADDR=$1
#BAG_NAME=$2

OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/batch_test_match/6_12_cloudy_building_near.bag
BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/6_14_morning_sunny_building_near.bag
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/office_1.bag
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

GLO_MATCH_ADDR=${EXE_ROOT}/devel/lib/global_match_test/global_match_test 

${GLO_MATCH_ADDR} ${OUT_ADDR} ${OUT_ADDR}/opti_chamo_1000.map ${BAG_NAME} img maplab
