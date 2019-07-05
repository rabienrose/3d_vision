BAG_NAME1=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/long_dis_test/combine1/1000_chamo.map
BAG_NAME2=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/long_dis_test/combine2/1001_chamo.map
DESC_FILE1=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/long_dis_test/combine1
DESC_FILE2=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/long_dis_test/combine2
OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/long_dis_test/combine1/chamo_out.map
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
