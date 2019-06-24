BAG_NAME1=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/batch_test_match/6_12_cloudy_building_near.bag/opti_chamo_1000.map
BAG_NAME2=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/batch_test_match/6_14_sunset_sunny_building_near_2.bag/opti_chamo_1000.map
DESC_FILE1=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/batch_test_match/6_12_cloudy_building_near.bag
DESC_FILE2=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/batch_test_match/6_14_sunset_sunny_building_near_2.bag
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/batch_test_match/combine/chamo.map
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

EXE_ADDR=${EXE_ROOT}/devel/lib/vis_combine/vis_combine

${EXE_ADDR} \
    --map1_addr=${BAG_NAME1} \
    --map2_addr=${BAG_NAME2} \
    --desc1_addr=${DESC_FILE1} \
    --desc2_addr=${DESC_FILE2} \
    --output_addr=${OUT_ADDR} \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
