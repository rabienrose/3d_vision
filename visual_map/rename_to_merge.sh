ROOT_DIR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_combine_garage
TARGET_DIR=${ROOT_DIR}/combine
mkdir ${TARGET_DIR}
DIRS=`ls ${ROOT_DIR}`
COUNT=0;
for i in ${DIRS}
do
   cp ${ROOT_DIR}/${i}/index.dat ${TARGET_DIR}/chamo_${COUNT}.map.desc
   cp ${ROOT_DIR}/${i}/imu_chamo.map ${TARGET_DIR}/chamo_${COUNT}.map
   cp ${ROOT_DIR}/${i}/words_projmat.dat ${TARGET_DIR}/words_projmat.dat
   ((COUNT++))
done
