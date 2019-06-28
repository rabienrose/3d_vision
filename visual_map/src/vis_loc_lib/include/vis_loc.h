#pragma once
#include <global_match/global_match.h>

#include <inverted-multi-index/inverted-multi-index.h>
#include <opencv2/core/eigen.hpp>
#include <orb_slam_lib/sim3_match.h>

#include "Converter.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "System.h"
#include "Tracking.h"
#include "read_write_data_lib/read_write.h"
#include "ros/ros.h"
// #include "g2o/types/types_seven_dof_expmap.h"

#define MAX_NO_ALINED_COUNT 10000
namespace visual_loc {

class VisualLocalization {
public:
    VisualLocalization();

    bool AddImage(cv::Mat& img,
                  std::string& img_name,
                  double& time_stamp,
                  Eigen::Vector3d& output_posi);

    void LoadMapLabMap();

    bool SetInit(const std::string& dir_name);

    Eigen::Vector3d GetGlobalPosition(string& img_name);

    Eigen::Vector3d GetLocalPosition();

    bool AlignLocalMap(Eigen::Matrix4d& frame_pose,
                       std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                       std::vector<ORB_SLAM2::KeyFrame*>& covisible_kf,
                       std::vector<ORB_SLAM2::MapPoint*>& mps_all);


    bool AlignLocalMapByMapPoimt(Eigen::Matrix4d& frame_pose,
                                 std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                                 std::vector<ORB_SLAM2::MapPoint*>& mps_all,
                                 ORB_SLAM2::Frame& frame,
                                 std::map<int, Eigen::Vector3d>& local_kp_global_mp);

    void TransformPoseUseSim3(const Eigen::Matrix4d& sim3,
                              const double scale,
                              Eigen::Matrix4d& in_pose,
                              Eigen::Matrix4d& out_pose);

    void TransformPositionUseSim3(const Eigen::Matrix4d& sim3,
                                  const double scale,
                                  Eigen::Vector3d& in_position,
                                  Eigen::Vector3d& out_position);

    bool CheckAlign(std::vector<Eigen::Vector3d>& global,
                    std::vector<Eigen::Vector3d>& local,
                    const Eigen::Matrix4d& T12i_eig,
                    const double& scale,
                    const double& threshold,
                    bool need_consistency);

    bool UpdateAlignHistory(bool b_check);

    bool ResetInit();

    void GetNNearestKF(int frame_id,
                       int N,
                       std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                       std::vector<ORB_SLAM2::KeyFrame*>& covisible_kf);

    void UpdateLocalMap(const Eigen::Matrix4d& T12i_eig,
                        const double& scale,
                        Eigen::Matrix4d& frame_pose,
                        std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                        std::vector<ORB_SLAM2::MapPoint*>& mps_all);

    void UpdateGlobalLocalMatches(std::vector<ORB_SLAM2::MapPoint*> mps_local);

    bool ComputeSim3Ransac(std::vector<Eigen::Vector3d>& P1,
                           std::vector<Eigen::Vector3d>& P2,
                           Eigen::Matrix4d& T12i_eig,
                           double& scale);

    int CheckInliers(std::vector<Eigen::Vector3d>& P1,
                     std::vector<Eigen::Vector3d>& P2,
                     const Eigen::Matrix4d& T12i_eig,
                     const double& scale,
                     std::vector<bool> &b_inliners);

private:
    ORB_SLAM2::System* mpsys;
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_;
    std::string workspace;
    Eigen::MatrixXf projection_matrix_;
    std::vector<Eigen::Vector3d> mp_posis;
    bool align_init;
    bool track_init;
    bool former_match;
    int no_align_count;
    std::map<int, Eigen::Matrix4d> global_pose;
    std::vector<Eigen::Vector3d> local_pose;
    bool check_align_history[3];
    std::map<int, ORB_SLAM2::MapPoint*> global_id_local_pointer;

public:
    std::vector<Eigen::Vector3d> local_posis;
    std::vector<Eigen::Vector3d> global_posis;
    std::vector<Eigen::Matrix4d> local_poses;
    std::vector<Eigen::Matrix4d> global_poses;
    std::vector<Eigen::Vector3d> mp_before;
    std::vector<Eigen::Vector3d> mp_after;
};
}  // namespace visual_loc
