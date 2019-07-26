ROOT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_all
CONF_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_720_configs
BAG_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/all
TARGET_DIR=${ROOT_ADDR}/combine
rm -rf ${ROOT_ADDR}
mkdir ${ROOT_ADDR}

map_bag_list=`ls ${BAG_ROOT}`

for map_bag_addr in ${map_bag_list}
do
    map_name=`basename "${map_bag_addr}"`
    echo create map: ${map_name}
    mkdir ${ROOT_ADDR}/${map_name}
    cp ${CONF_ADDR}/* ${ROOT_ADDR}/${map_name}
    bash ./build_iphone_out.sh ${ROOT_ADDR}/${map_name} ${BAG_ROOT}/${map_bag_addr}
done
bash ./rename_to_merge.sh ${ROOT_ADDR} opti_1000_chamo.map
bash ./run_vis_combine.sh ${TARGET_DIR}
bash ./run_optimization.sh "pose graph" ${TARGET_DIR} chamo_out.map
bash ./run_optimization.sh "BA" ${TARGET_DIR} graph_chamo_out.map
bash ./run_optimization.sh "BA" ${TARGET_DIR} opti_graph_chamo_out.map
#bash ./run_culling.sh "project" ${TARGET_DIR} opti_opti_graph_chamo_out.map
#bash ./run_culling.sh "mp" ${TARGET_DIR} culling_opti_opti_graph_chamo_out.map
#bash ./run_culling.sh "frame" ${TARGET_DIR} culling_culling_opti_graph_chamo_out.map
#bash ./run_optimization.sh "BA" ${TARGET_DIR} culling_culling_culling_opti_opti_graph_chamo_out.map
