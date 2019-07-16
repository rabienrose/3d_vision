#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
namespace OptimizerTool
{
    void optimize_lidar_pose(std::string res_root);
    void optimize_gps_pose(std::string map_addr, std::string map_name);
    void optimize_imu(std::string map_addr, std::string map_name);
    void optimize_sim3_graph(std::string map_addr, std::string map_name);
    void findFramePoseByName(std::vector<std::string>& names, int& re_id, std::string query_name);
    void optimize_sim3_graph( std::vector<Eigen::Vector3d>& gps_alin, std::vector<int>& gps_inlers,
        std::vector<Eigen::Matrix4d>& poses_out, std::vector<Eigen::Matrix4d>& poses_in,
        std::vector<Eigen::Matrix4d>& T_1_to_2_list, std::vector<double>& scale_1_to_2_list, 
        std::vector<int>& graph_v1_list, std::vector<int>& graph_v2_list, bool input_is_sim=false
    );
    void local_BA(std::string map_addr, std::string map_name);
}