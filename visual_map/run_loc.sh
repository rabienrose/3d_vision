OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/bag_match_lidar/test_720_30_2
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/bag_match_lidar/06-12-14-09-52.bag
BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/office_1.bag
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

LOC_ADDR=${EXE_ROOT}/devel/lib/vis_loc_lib/vis_loc_lib_test
${LOC_ADDR} ${OUT_ADDR} ${BAG_NAME}
