#MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_builddown/combine
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

TYPE=$1
MAP_ADDR=$2
MAP_NAME=$3
OPTIMIZATION_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
${OPTIMIZATION_ADDR} \
    --map_addr=${MAP_ADDR} \
    --map_name=${MAP_NAME} \
    --opti_type="${TYPE}" \
    --opti_count=100 \
    --gps_weight=0.0000000000001 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
