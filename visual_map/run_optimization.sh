MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_combine_garage/combine
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

OPTIMIZATION_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
${OPTIMIZATION_ADDR} \
    --map_addr=${MAP_ADDR} \
    --map_name=opti_graph_chamo_out.map \
    --opti_type="BA" \
    --opti_count=100 \
    --gps_weight=0.000001 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
