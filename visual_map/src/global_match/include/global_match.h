#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <inverted-multi-index/inverted-index.h>
#include <inverted-multi-index/inverted-multi-index.h>
#include <inverted-multi-index/kd-tree-index.h>

#include <maplab-common/binary-serialization.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "inverted-multi-index/inverted_multi_index.pb.h"
#include "read_write_data_lib/read_write.h"


namespace ORB_SLAM2 {
    class Frame;
}

namespace chamo {
void LoadMap(std::string res_root, std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>>& index_,
    Eigen::MatrixXf& projection_matrix_);

void MatchImg(std::vector<Eigen::Vector3d>& mp_posis, std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>>& index_,
    Eigen::MatrixXf& projection_matrix_, ORB_SLAM2::Frame& frame, std::vector<int>& inliers_mp, std::vector<int>& inliers_kp,
    Eigen::Matrix4d& pose
);

}