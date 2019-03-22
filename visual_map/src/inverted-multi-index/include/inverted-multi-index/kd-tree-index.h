#ifndef MATCHING_BASED_LOOPCLOSURE_KD_TREE_INDEX_H_
#define MATCHING_BASED_LOOPCLOSURE_KD_TREE_INDEX_H_

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <nabo/nabo.h>
DECLARE_double(lc_knn_max_radius);
namespace loop_closure {
namespace kd_tree_index {
template <int kDimVectors>
class KDTreeIndex {
 public:
  typedef Eigen::Matrix<float, kDimVectors, 1> DescriptorType;
  typedef Eigen::Matrix<float, kDimVectors, Eigen::Dynamic>
      DescriptorMatrixType;
  typedef Nabo::NearestNeighbourSearch<float> NNSearch;
  // Switch touch statistics (NNSearch::TOUCH_STATISTICS) off for performance.
  static constexpr int kCollectTouchStatistics = 0;
  // Kd-tree search options. ALLOW_SELF_MATCH means that a point which is
  // equal to the query will be returned in the results.
  static constexpr unsigned kSearchOptionsDefault =
      NNSearch::ALLOW_SELF_MATCH | NNSearch::SORT_RESULTS;
  // Epsilon approximation factor for kd-tree backtracking.
  static constexpr float kSearchNNEpsilon = 0.1;

  KDTreeIndex() {}

  inline void Clear() {
    index_data_.resize(Eigen::NoChange, 0);
    index_.reset();
    pending_descriptor_blocks_.clear();
  }

  inline int GetNumDescriptorsInIndex() const {
    int num_descriptors = index_data_.cols();
    for (const std::shared_ptr<Eigen::MatrixXf>& pending_block :
         pending_descriptor_blocks_) {
      num_descriptors += pending_block->cols();
    }
    return num_descriptors;
  }

  // Adds descriptors to an internal waiting list. These descriptors will be
  // added on the next time the index is queried.
  void AddDescriptors(const DescriptorMatrixType& descriptors, int frame_id, int track_id) {
    pending_descriptor_blocks_.push_back(
        aligned_shared<Eigen::MatrixXf>(descriptors));
    frame_indice.push_back(frame_id);
    track_indice.push_back(track_id);
  }
  
  int get_desc_frameid(int desc_id){
      if(desc_id<frame_indice.size()){
          return frame_indice[desc_id];
      }else{
          std::cout<<"no frame id!!"<<std::endl;
          return -1;
      }
  }
  
  int get_desc_trackid(int desc_id){
      if(desc_id<track_indice.size()){
          return track_indice[desc_id];
      }else{
          std::cout<<"no track id!!"<<std::endl;
          return -1;
      }
  }

  void RefreshIndex() const {
    if (pending_descriptor_blocks_.empty())
      return;

    int total_num_descriptors_to_add = 0;
    for (const std::shared_ptr<Eigen::MatrixXf>& descriptor_block :
         pending_descriptor_blocks_) {
      CHECK(descriptor_block != nullptr);
      total_num_descriptors_to_add += descriptor_block->cols();
    }
    int old_num_descriptors = index_data_.cols();
    int new_num_descriptors =
        total_num_descriptors_to_add + old_num_descriptors;

    index_data_.conservativeResize(kDimVectors, new_num_descriptors);

    int curr_offset = old_num_descriptors;
    for (const std::shared_ptr<Eigen::MatrixXf>& descriptor_block :
         pending_descriptor_blocks_) {
      const DescriptorMatrixType& descriptors = *descriptor_block;
      int num_descriptors = descriptors.cols();
      index_data_.block(0, curr_offset, kDimVectors, num_descriptors) =
          descriptors;
      curr_offset += num_descriptors;
    }

    if (index_data_.cols() == 0) {
      pending_descriptor_blocks_.clear();
      index_.reset();
      return;
    }
    index_.reset(
        NNSearch::createKDTreeLinearHeap(
            index_data_, kDimVectors, kCollectTouchStatistics));
    pending_descriptor_blocks_.clear();
  }

  // Finds the n nearest neighbors for a given query feature.
  // This function is thread-safe.
  template <typename DerivedQuery, typename DerivedIndices,
            typename DerivedDistances>
  inline void GetNNearestNeighbors(
      Eigen::MatrixBase<DerivedQuery>& query_features, int num_neighbors,
      Eigen::MatrixBase<DerivedIndices>& indices,
      Eigen::MatrixBase<DerivedDistances>& distances) {
    CHECK_EQ(indices.rows(), num_neighbors)
        << "The indices parameter must be pre-allocated to hold all results.";

    CHECK_EQ(distances.rows(), num_neighbors)
        << "The distances parameter must be pre-allocated to hold all results.";

    // Lazy refresh of the index if more data was added in the meantime.
    RefreshIndex();
    NNSearch::IndexVector indices_1;
    indices_1.resize(num_neighbors, 1);
    NNSearch::Vector distances_1;
    distances_1.resize(num_neighbors, 1);
    index_->knn(
        query_features, indices_1, distances_1, num_neighbors, kSearchNNEpsilon,
        kSearchOptionsDefault, FLAGS_lc_knn_max_radius);
    indices=indices_1;
    distances=distances_1;
    //std::cout<<query_features.transpose()<<std::endl;
    //std::cout<<indices_1.transpose()<<std::endl;
  }
  std::vector<int> frame_indice;
  std::vector<int> track_indice;

 protected:
  mutable std::shared_ptr<NNSearch> index_;
  mutable Eigen::MatrixXf index_data_;
  mutable std::vector<std::shared_ptr<Eigen::MatrixXf> >
      pending_descriptor_blocks_;
};
}  // namespace kd_tree_index
}  // namespace loop_closure

#endif  // MATCHING_BASED_LOOPCLOSURE_KD_TREE_INDEX_H_
