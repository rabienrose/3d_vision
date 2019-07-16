OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/demo_loc
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/bag_match_lidar/06-12-14-09-52.bag
BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/6_19_cloudy_building_up.bag
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

LOC_ADDR=${EXE_ROOT}/devel/lib/vis_loc_lib/vis_loc_lib_test
${LOC_ADDR} --bag_addr=${BAG_NAME} --output_addr=${OUT_ADDR}/ --voc_addr=${OUT_ADDR}/FreakAll.bin --camera_config=${OUT_ADDR}/camera_config.txt --use_orb=false --feature_count=2000 --feature_scale_factor=1.2 --feature_level=8 --min_match_count=200 --max_step_KF=15 --v=0 --logtostderr=true --colorlogtostderr=true
