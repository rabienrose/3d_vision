#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include "NavState.h"

void show_pose_as_marker(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& pose_vec, std::string topic);

void show_mp_as_cloud(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& mp_posis, Eigen::Matrix3d Rwi_, std::string topic);