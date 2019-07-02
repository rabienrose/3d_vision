MAP_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/highway/combine1
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

OPTIMIZATION_ADDR=${EXE_ROOT}/devel/lib/optimizer_tool/optimizer_tool_test
${OPTIMIZATION_ADDR} \
    --map_addr=${MAP_ADDR} \
    --map_name=chamo_out.map \
    --opti_type="pose graph" \
    --opti_count=100 \
    --gps_weight=0.000000001 \
