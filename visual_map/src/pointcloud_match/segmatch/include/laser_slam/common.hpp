#ifndef LASER_SLAM_COMMON_HPP_
#define LASER_SLAM_COMMON_HPP_

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <kindr/minimal/rotation-quaternion.h>
#include <kindr/minimal/angle-axis.h>
#include "kindr/minimal/quat-transformation.h"

namespace laser_slam {
template<template<typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type> >;

typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef SE3::Rotation SO3;
typedef long long Time;
typedef std::map<Time, SE3> Trajectory;
typedef size_t Key;

struct Pose {
  /// \brief Absolute transform.
  SE3 T_w;
  /// \brief Time stamp.
  Time time_ns;
  /// \brief Node key.
  Key key;
};

struct RelativePose {
  /// \brief Relative transform.
  SE3 T_a_b;
  /// \brief Time stamp at frame A.
  Time time_a_ns;
  /// \brief Time stamp at frame B.
  Time time_b_ns;
  /// \brief Prior node key.
  Key key_a;
  /// \brief Posterior node key.
  Key key_b;
  unsigned int track_id_a;
  unsigned int track_id_b;
};

typedef std::vector<std::string> StringRow;
typedef std::vector<std::vector<std::string> > StringMatrix;


static void writeCSV(const StringMatrix& string_matrix, const std::string& filename) {
  CHECK_GE(string_matrix.size(), 1) << "Provided matrix of strings had no entries.";
  std::ofstream out_file_stream;
  out_file_stream.open(filename.c_str());

  // Iterate over the rows of the string matrix and write comma-separated fields.
  for (StringMatrix::const_iterator it = string_matrix.begin(); it != string_matrix.end(); ++it) {
    CHECK_GE(it->size(), 1) << "String matrix row has no entries.";
    out_file_stream << it->at(0u);
    for (size_t i = 1u; i < it->size(); ++i) {
      out_file_stream << "," << it->at(i);
    }
    out_file_stream << std::endl;
  }
  out_file_stream.close();
}

static void writeEigenMatrixXdCSV(const Eigen::MatrixXd& matrix, const std::string& filename) {
  StringMatrix string_matrix;
  string_matrix.reserve(matrix.rows());
  StringRow string_row;
  string_row.reserve(matrix.cols());
  for (size_t i = 0u; i < matrix.rows(); ++i) {
    string_row.clear();
    for (size_t j = 0u; j < matrix.cols(); ++j) {
      string_row.push_back(std::to_string(matrix(i,j)));
    }
    string_matrix.push_back(string_row);
  }
  writeCSV(string_matrix, filename);
}

static double distanceBetweenTwoSE3(const SE3& pose1, const SE3& pose2) {
  return std::sqrt(
      (pose1.getPosition()(0) - pose2.getPosition()(0)) *
      (pose1.getPosition()(0) - pose2.getPosition()(0)) +
      (pose1.getPosition()(1) - pose2.getPosition()(1)) *
      (pose1.getPosition()(1) - pose2.getPosition()(1)) +
      (pose1.getPosition()(2) - pose2.getPosition()(2)) *
      (pose1.getPosition()(2) - pose2.getPosition()(2)));
}

} // namespace laser_slam

#endif /* LASER_SLAM_COMMON_HPP_ */
