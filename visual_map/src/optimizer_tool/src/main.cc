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
    
    std::string posi_addr=res_root+"/posi_alin.txt";
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    show_mp_as_cloud(mp_posis, Rwi_, "chamo_target");
    
    std::vector<Eigen::Vector3d> lidar_posis;
    std::vector<Eigen::Quaterniond> lidar_dirs;
    std::vector<double> lidar_time;
    std::string lidar_addr=res_root+"/lidar_trajectory.txt";
    CHAMO::read_lidar_pose(lidar_addr, lidar_dirs, lidar_posis, lidar_time);
    show_mp_as_cloud(lidar_posis, Rwi_, "/chamo/gps");
    //ros::spin();
}