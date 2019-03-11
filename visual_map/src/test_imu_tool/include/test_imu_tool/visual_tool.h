#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include "NavState.h"

void show_pose_as_marker(std::vector<ORB_SLAM2::NavState>& states, Eigen::Matrix3d Rwi_, std::string topic);

void show_mp_as_cloud(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& mp_posis, Eigen::Matrix3d Rwi_, std::string topic);