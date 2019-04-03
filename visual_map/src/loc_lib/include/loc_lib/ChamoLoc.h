#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <loc_lib/LocalizationAPI.h>
#include <rovio/RovioFilter.hpp>
#include <inverted-multi-index/inverted-index.h>
#include <inverted-multi-index/inverted-multi-index.h>
#include <inverted-multi-index/kd-tree-index.h>

#include <maplab-common/binary-serialization.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "inverted-multi-index/inverted_multi_index.pb.h"
#include "read_write_data_lib/read_write.h"
#include "orb_slam_lib/two_frame_pose.h"

namespace wayz {

class ChamoLoc: public LocalizationAPI {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void StartLocalization(const std::string& filename);
    
    void AddGPS(const double timestamp, const Eigen::Vector3d& LatLonHei){};//LatLonHei means Lattitude Longitude and Height
  
    void AddImage(const double timestamp,const int camera_id, const cv::Mat& Img);
    
    void AddIMU(const double timestamp, const Eigen::Vector3d& Accl, const Eigen::Vector3d& Gyro);

    void AddMap(std::stringstream& pointer_address){};

    void AddMap(const std::string& folder_path);
    
    void Shutdown();

    bool QueryPose(const double timestamp, Eigen::Vector3d& Pos, Eigen::Vector3d& Vel, Eigen::Quaterniond& Ori) const;
private:
    //kMaxNumFeatures, kPyramidLevels, kFeaturePatchSizePx, kNumCameras, kLocalizationMode
    typedef rovio::RovioFilter<rovio::FilterState<25, 4, 6, 1, 1>> FilterType; 
    std::shared_ptr<FilterType> mpFilter_;
    
    typedef typename FilterType::mtPrediction::mtMeas mtPredictionMeas;
    mtPredictionMeas predictionMeas_;

    typedef typename std::tuple_element<0, typename FilterType::mtUpdates>::type mtImgUpdate;
    mtImgUpdate *mpImgUpdate_;

    typedef typename std::tuple_element<1, typename FilterType::mtUpdates>::type mtPoseUpdate;
    mtPoseUpdate *mpPoseUpdate_;

    typedef typename mtImgUpdate::mtMeas mtImgMeas;
    mtImgMeas imgUpdateMeas_;
    typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
    mtPoseMeas poseUpdateMeas_;

    typedef typename std::tuple_element<2, typename FilterType::mtUpdates>::type mtVelocityUpdate;
    typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;
    mtVelocityMeas velocityUpdateMeas_;

    struct FilterInitializationState {
        FilterInitializationState() : WrWM_(V3D::Zero()), state_(State::WaitForInitUsingAccel) {}

        enum class State {
        // Initialize the filter using accelerometer measurement on the next
        // opportunity.
        WaitForInitUsingAccel,
        // Initialize the filter using an external pose on the next opportunity.
        WaitForInitExternalPose,
        // The filter is initialized.
        Initialized
        } state_;

        // Buffer to hold the initial pose that should be set during initialization
        // with the state WaitForInitExternalPose.
        V3D WrWM_;
        QPD qMW_;

        explicit operator bool() const { return isInitialized(); }

        bool isInitialized() const { return (state_ == State::Initialized); }
    };
    
    bool updateFilter();

    FilterInitializationState init_state_;    
    
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> mp_posis;
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_; 
    Eigen::MatrixXf projection_matrix_;
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    cv::Mat cam_inter_cv;
    cv::Mat cam_distort_cv;
    std::map<double, Eigen::Vector3d> posi_list;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> posi_vec;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> posi_match_vec;
    std::vector<Eigen::Quaterniond> rot_match_vec;
    std::map<double, Eigen::Quaterniond> rot_list;
    std::map<double, Eigen::Vector3d> vel_list;
    std::map<double, Eigen::Matrix4d> gt_list;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> posi_loc_vec;
    std::vector<Eigen::Quaterniond> rot_loc_vec;
    std::vector<double> timestamp_list;

};

}

