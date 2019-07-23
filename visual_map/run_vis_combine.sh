#MAP_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_builddown/combine
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map
EXE_ADDR=${EXE_ROOT}/devel/lib/vis_combine/vis_combine

MAP_ROOT=$1

${EXE_ADDR} \
    --map_root_addr=${MAP_ROOT} \
    --v=0 \
    --logtostderr=true \
    --colorlogtostderr=true
