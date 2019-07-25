#OUT_ADDR=$1
#BAG_NAME=$2

OUT_ADDR=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/acuu_test/666
BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/acuu_test/20190724110707_jt_01.bag
#BAG_NAME=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_test/6_14_sunset_sunny/office_1.bag
EXE_ROOT=/home/chamo/Documents/work/3d_vision/visual_map

GLO_MATCH_ADDR=${EXE_ROOT}/devel/lib/global_match_test/global_match_test 

${GLO_MATCH_ADDR} --res_root=${OUT_ADDR} --map_name=${OUT_ADDR}/opti_1000_chamo.map --bag_addr=${BAG_NAME} --img_topic=camera/left/image_raw --match_method=maplab --t_c_g_x=0.1 --t_c_g_y=0.0 --t_c_g_z=0.0
