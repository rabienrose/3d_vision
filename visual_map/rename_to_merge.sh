ROOT_DIR=$1
MAP_NAME=$2
TARGET_DIR=${ROOT_DIR}/combine
mkdir ${TARGET_DIR}
DIRS=`ls ${ROOT_DIR}`
COUNT=0;
for i in ${DIRS}
do
   cp ${ROOT_DIR}/${i}/index.dat ${TARGET_DIR}/chamo_${COUNT}.map.desc
   cp ${ROOT_DIR}/${i}/${MAP_NAME} ${TARGET_DIR}/chamo_${COUNT}.map
   cp ${ROOT_DIR}/${i}/words_projmat.dat ${TARGET_DIR}/words_projmat.dat
   ((COUNT++))
done
