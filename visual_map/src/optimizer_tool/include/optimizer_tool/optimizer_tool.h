#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
namespace OptimizerTool
{
    void optimize_true_pose(std::string res_root);
    void optimize_imu(std::string res_root);
}