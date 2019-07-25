IMG_ADDR=$1
#BAG_NAME=$2

OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/merge/workspace_indoor_office/combine
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

GLO_MATCH_ADDR=${EXE_ROOT}/devel/lib/loc_a_img/loc_a_img 

${GLO_MATCH_ADDR} --res_root=${OUT_ADDR} --map_name=${OUT_ADDR}/opti_opti_chamo_out.map --bag_addr=${IMG_ADDR} --match_method=maplab
