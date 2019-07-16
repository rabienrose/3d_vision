ROOT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_combine_garage
CONF_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_720_configs
BAG_ROOT=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/7_10_garage

#rm -rf ${ROOT_ADDR}
#mkdir ${ROOT_ADDR}

map_bag_list=`ls ${BAG_ROOT}`

for map_bag_addr in ${map_bag_list}
do
    map_name=`basename "${map_bag_addr}"`
    echo create map: ${map_name}
    mkdir ${ROOT_ADDR}/${map_name}
    cp ${CONF_ADDR}/* ${ROOT_ADDR}/${map_name}
    bash ./build_map_iphone.sh ${ROOT_ADDR}/${map_name} ${BAG_ROOT}/${map_bag_addr}
done
