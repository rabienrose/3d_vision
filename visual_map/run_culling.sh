#MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_builddown/combine
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

TYPE=$1
MAP_ADDR=$2
MAP_NAME=$3
CULLING_ADDR=${EXE_ROOT}/devel/lib/culling_map/culling_map
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test

${CULLING_ADDR} \
    --map_addr=${MAP_ADDR} \
    --map_name=${MAP_NAME} \
    --culling_type=${TYPE} \
    --max_repro_err=3 \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
    
#${INDEX_ADDR} ${MAP_ADDR} culling_chamo_out.map
