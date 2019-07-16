DIR=/home/chamo/sortData
DIRS=`ls ${DIR}`

for i in ${DIRS}
do
   cp ${DIR}/${i}/* ${DIR}
done
