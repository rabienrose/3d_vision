OUT_ADDR=$1
BAG_NAME=$2




#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/office_2.bag
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/6_19_cloudy_building_near.bag
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_imu/07-09-17-46-43.bag
#OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_android
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

echo working directory ${OUT_ADDR}
echo bag address ${BAG_NAME}

BAG_EXTRACT_ADDR=${EXE_ROOT}/devel/lib/bag_tool/bag_tool_exe
ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti
CONV_MAP_ADDR=${EXE_ROOT}/devel/lib/convert_to_visual_map/convert_to_visual_map
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test
OPTI_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
SEG_ADDR=${EXE_ROOT}/devel/lib/seg_traj/seg_traj
DSO_ADDR=${EXE_ROOT}/devel/lib/dso/dso_test 
POSE_EXTRACTOR=${EXE_ROOT}/devel/lib/global_match_test/global_match_test 

mkdir ${OUT_ADDR}/images
${BAG_EXTRACT_ADDR} --bag_addr=${BAG_NAME} --out_dir=${OUT_ADDR} --img_topic=img --imu_topic=imu --gps_topic=gps --isExtractImage=false
${ORB_SLAM_ADDR} --bag_addr=${BAG_NAME} --output_addr=${OUT_ADDR}/ --voc_addr=${OUT_ADDR}/FreakAll.bin --camera_config=${OUT_ADDR}/camera_config.txt --image_topic=img --min_frame=0 --max_frame=10000 --step_frame=1 --use_orb=false --feature_count=2000 --feature_scale_factor=1.2 --feature_level=8 --min_match_count=100 --max_step_KF=15 --v=0 --logtostderr=true --colorlogtostderr=true
${CONV_MAP_ADDR} --res_root=${OUT_ADDR} --map_name=chamo.map visualmap
${OPTI_ADDR} --map_addr=${OUT_ADDR} --map_name=chamo.map --opti_type=imu --opti_count=100 --gps_weight=0.00000001
${INDEX_ADDR} ${OUT_ADDR} imu_chamo.map
