#include <string>
#include <fstream>
#include <memory>
#include "optimizer_tool/optimizer_tool.h"
#include "test_imu_tool/visual_tool.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"
#include "test_imu_tool/visual_tool.h"


int main(int argc, char* argv[]) {
    std::string res_root=argv[1];
    std::cout<<res_root<<std::endl;
    visualization::RVizVisualizationSink::init();
    
    Eigen::Matrix3d Rwi_=Eigen::Matrix3d::Identity();
    
    
    
    
    //OptimizerTool::optimize_imu(res_root);
    OptimizerTool::optimize_true_pose(res_root);
    //show_pose_as_marker(pose_vec, "pose_cam");
    
    std::string posi_addr=res_root+"/mp_posi_opt.txt";
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    show_mp_as_cloud(mp_posis, Rwi_, "chamo_target");
    
    std::string gps_alin_addr=res_root+"/gps_alin.txt";
    std::vector<int> gps_inliers;
    std::vector<Eigen::Vector3d> gps_alins;
    CHAMO::read_gps_alin(gps_alin_addr, gps_alins, gps_inliers);
    show_mp_as_cloud(gps_alins, Rwi_, "/chamo/gps");
    ros::spin();
}