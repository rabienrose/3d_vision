#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
namespace OptimizerTool
{
    void optimize_lidar_pose(std::string res_root);
    void optimize_gps_pose(std::string res_root);
    void optimize_imu(std::string res_root);
    void optimize_sim3_graph(std::string res_root);
    void findFramePoseByName(std::vector<std::string>& names, int& re_id, std::string query_name);
}