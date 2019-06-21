GARAGE_12=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/6_12_cloudy_garage_1.bag

BUILDUP_12=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/6_12_cloudy_building_up_1.bag
BUILDNEAR_12=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/6_12_cloudy_building_near.bag
OFFICE_12=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_12_cloudy/6_12_cloudy_office_2.bag

BUILDUP_19=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/6_19_cloudy_building_up.bag
BUILDNEAR_19=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/6_19_cloudy_building_near.bag
OFFICE_19=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_19_cloudy/6_19_cloudy_office.bag

BUILDUP_14_M=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/6_14_morning_sunny_building_up.bag
BUILDNEAR_14_M=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/6_14_morning_sunny_building_near.bag
OFFICE_14_M=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_morning_sunny/6_14_morning_sunny_office_2.bag

BUILDUP_14_A=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/6_14_sunset_sunny_building_up.bag
BUILDNEAR_14_A=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/6_14_sunset_sunny_building_near_2.bag
OFFICE_14_A=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/6_14_sunset_sunny_office_1.bag

BUILDUP_11_A=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_11_afternoon/6_11_afternoon_building_up_1.bag
OFFICE_11_A=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_11_afternoon/6_11_afternoon_office_1.bag

ROOT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/batch_test_match
CONF_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_720_configs

#rm -rf ${ROOT_ADDR}
#mkdir ${ROOT_ADDR}

map_bag_list=( \
${BUILDUP_12} \
${BUILDNEAR_12} \
${OFFICE_12} \
${BUILDUP_14_A} \
${BUILDNEAR_14_A} \
${OFFICE_14_A} \
)

for map_bag_addr in "${map_bag_list[@]}"
do
    map_name=`basename "${map_bag_addr}"`
    echo create map: ${map_name}
    #mkdir ${ROOT_ADDR}/${map_name}
    #cp ${CONF_ADDR}/* ${ROOT_ADDR}/${map_name}
    #bash ./build_map_iphone.sh ${ROOT_ADDR}/${map_name} ${map_bag_addr}
done

map_list=( \
${BUILDUP_12} \
${BUILDNEAR_12} \
${OFFICE_12} \
${BUILDUP_12} \
${BUILDNEAR_12} \
${OFFICE_12} \
${BUILDUP_12} \
${BUILDNEAR_12} \
${OFFICE_12} \
${BUILDUP_14_A} \
${BUILDNEAR_14_A} \
${OFFICE_14_A} \
${BUILDUP_14_A} \
${OFFICE_14_A} \
)

loc_bag_list=( \
${BUILDUP_19} \
${BUILDNEAR_19} \
${OFFICE_19} \
${BUILDUP_14_M} \
${BUILDNEAR_14_M} \
${OFFICE_14_M} \
${BUILDUP_14_A} \
${BUILDNEAR_14_A} \
${OFFICE_14_A} \
${BUILDUP_14_M} \
${BUILDNEAR_14_M} \
${OFFICE_14_M} \
${BUILDUP_11_A} \
${OFFICE_11_A} \
)

COUNTER=0
for map_bag_addr in "${map_list[@]}"
do
    echo ${COUNTER}
    loc_name=`basename "${loc_bag_list[${COUNTER}]}"`
    map_name=`basename "${map_bag_addr}"`
    echo ${map_name}
    echo ${loc_name}
    map_workspace=${ROOT_ADDR}/${map_name}
    loc_workspace=${map_workspace}/${loc_name}
    echo ${map_workspace}
    echo ${loc_workspace}
    mkdir ${loc_workspace}
    cp ${map_workspace}/words_projmat.dat ${loc_workspace}
    cp ${map_workspace}/camera_config_loc.txt ${loc_workspace}/camera_config.txt
    cp ${map_workspace}/index.dat ${loc_workspace}
    cp ${map_workspace}/opti_chamo_1000.map ${loc_workspace}
    cp ${map_workspace}/image_conf.txt ${loc_workspace}
    
    bash ./test_global_match.sh ${loc_workspace} ${loc_bag_list[${COUNTER}]}
    COUNTER=$[$COUNTER +1]
done
