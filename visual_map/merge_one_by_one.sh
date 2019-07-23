ROOT_DIR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_builddown
TARGET_DIR=${ROOT_DIR}/combine1
rm -rf ${TARGET_DIR}/*
mkdir ${TARGET_DIR}
DIRS=`ls ${ROOT_DIR}`

EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map
INDEX_ADDR=${EXE_ROOT}/devel/lib/create_desc_index/create_desc_index_test

COUNT=0;
for i in ${DIRS}
do
    if [ ${i: -4} == ".bag" ]
    then
        cp ${ROOT_DIR}/${i}/index.dat ${TARGET_DIR}/chamo_${COUNT}.map.desc
        cp ${ROOT_DIR}/${i}/opti_1000_chamo.map ${TARGET_DIR}/chamo_${COUNT}.map
        cp ${ROOT_DIR}/${i}/words_projmat.dat ${TARGET_DIR}/words_projmat.dat
        ((COUNT++))
        if [ ${COUNT} -gt 1 ]
        then
            bash ./run_vis_combine.sh ${TARGET_DIR}
            bash ./run_optimization.sh "pose graph" ${TARGET_DIR} chamo_out.map
            bash ./run_optimization.sh "BA" ${TARGET_DIR} graph_chamo_out.map
            ${INDEX_ADDR} ${TARGET_DIR} graph_chamo_out.map
            cp ${TARGET_DIR}/opti_graph_chamo_out.map ${ROOT_DIR}/opti_graph_chamo_out.map
            cp ${TARGET_DIR}/index.dat ${ROOT_DIR}/index.dat
            rm -rf ${TARGET_DIR}/*
            cp ${ROOT_DIR}/opti_graph_chamo_out.map ${TARGET_DIR}/chamo_0.map
            cp ${ROOT_DIR}/index.dat ${TARGET_DIR}/chamo_0.map.desc
        fi
    fi
    
done
