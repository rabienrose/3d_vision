BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/7_3/crowd_07-03-14-42-36.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_save_vm
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/mono_kitti

${ORB_SLAM_ADDR} \
    --bag_addr=${BAG_NAME} \
    --output_addr=${OUT_ADDR}/ \
    --voc_addr=${OUT_ADDR}/FreakAll.bin \
    --camera_config=${OUT_ADDR}/camera_config.txt \
    --image_topic=img \
    --min_frame=3000 \
    --max_frame=100000 \
    --step_frame=1 \
    --use_orb=false \
    --feature_count=2000 \
    --feature_scale_factor=1.2 \
    --feature_level=8 \
    --min_match_count=200 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
