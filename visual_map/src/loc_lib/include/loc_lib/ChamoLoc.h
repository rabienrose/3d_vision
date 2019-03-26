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
    
    virtual void StartLocalization(const std::string& filename);
    
    virtual void AddGPS(const double timestamp, const Eigen::Vector3d& LatLonHei){};//LatLonHei means Lattitude Longitude and Height
  
    virtual void AddImage(const double timestamp,const int camera_id, const cv::Mat& Img);
    
    virtual void AddIMU(const double timestamp, const Eigen::Vector3d& Accl, const Eigen::Vector3d& Gyro);

    virtual void AddMap(std::stringstream& pointer_address){};

    virtual void AddMap(const std::string& folder_path);
    
    virtual void Shutdown();

    virtual bool QueryPose(const double timestamp, Eigen::Vector3d& Pos, Eigen::Vector3d& Vel, Eigen::Quaterniond& Ori);
private:
    typedef rovio::RovioFilter<rovio::FilterState<25, 4, 6, 1, 1>> FilterType; 
    //kMaxNumFeatures, kPyramidLevels, kFeaturePatchSizePx, kNumCameras, kLocalizationMode
    std::shared_ptr<FilterType> mpFilter_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> mp_posis;
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_; 
    Eigen::MatrixXf projection_matrix_;
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    cv::Mat cam_inter_cv;
    cv::Mat cam_distort_cv;
    std::map<double, Eigen::Vector3d> posi_list;
    std::map<double, Eigen::Quaterniond> rot_list;
    std::map<double, Eigen::Vector3d> vel_list;

};

}

