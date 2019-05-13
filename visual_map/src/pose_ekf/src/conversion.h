#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

Eigen::Quaterniond euler2quaternion(Eigen::Vector3d euler);
Eigen::Matrix3d quaternion2mat(Eigen::Quaterniond q);
Eigen::Vector3d mat2euler(Eigen::Matrix3d m);
Eigen::Quaterniond mat2quaternion(Eigen::Matrix3d m);
Eigen::Matrix3d euler2mat(Eigen::Vector3d euler);
Eigen::Vector3d quaternion2euler(Eigen::Quaterniond q);