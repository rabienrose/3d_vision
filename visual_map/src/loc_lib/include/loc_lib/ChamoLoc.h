#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <loc_lib/LocalizationAPI.h>
#include <inverted-multi-index/inverted-index.h>
#include <inverted-multi-index/inverted-multi-index.h>
#include <inverted-multi-index/kd-tree-index.h>

#include <maplab-common/binary-serialization.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "inverted-multi-index/inverted_multi_index.pb.h"
#include "read_write_data_lib/read_write.h"
#include "Tracking.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "global_match/orb_match.h"
#include "global_match/global_match.h"
#include "ORBVocabulary.h"

namespace wayz {

struct raw_match
{
    double timestamp;
    int gmatchnum;
    double runtime;
};

class ChamoLoc: public LocalizationAPI {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void StartLocalization(const std::string& filename);
    
    void AddGPS(const double timestamp, const Eigen::Vector3d& LatLonHei){};//LatLonHei means Lattitude Longitude and Height
  
    void AddImage(const double timestamp,const int camera_id, const cv::Mat& Img);
    //add by yao 20190513
    void Debug_Image_pose(const double timestamp,const int camera_id, cv::Mat& img_distort);
    bool Debug_QueryPose(const double timestamp, Eigen::Vector3d& Pos, Eigen::Quaterniond& Ori);
    void Debug_Feature_pose(std::vector<Eigen::Vector3d>& vec);
    void Export_Raw_MatchFile(std::string& path);
    
    void AddIMU(const double timestamp, const Eigen::Vector3d& Accl, const Eigen::Vector3d& Gyro);

    void AddMap(std::stringstream& pointer_address){};

    void AddMap(const std::string& folder_path);
    
    void Shutdown();

    bool QueryPose(const double timestamp, Eigen::Vector3d& Pos, Eigen::Vector3d& Vel, Eigen::Quaterniond& Ori) const;
private:
    int UpdateByMap(cv::Mat Img, double timestamp, std::vector<cv::Point3f>& inliers_mp, std::vector<cv::Point2f>& inliers_kp);
    std::vector<Eigen::Vector3d> mp_posis;
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_; 
    Eigen::MatrixXf projection_matrix_;
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    cv::Mat cam_inter_cv;
    cv::Mat cam_distort_cv;
    ORB_SLAM2::ORBextractor* mpORBextractor;
    ORB_SLAM2::ORBVocabulary* mpVocabulary;
    std::map<double, Eigen::Vector3d> posi_list;
    std::vector<Eigen::Vector3d> posi_vec;
    std::vector<Eigen::Vector3d> posi_match_vec;
    std::vector<Eigen::Quaterniond> rot_match_vec;
    std::map<double, Eigen::Quaterniond> rot_list;
    std::map<double, Eigen::Vector3d> vel_list;
    std::map<double, Eigen::Matrix4d> gt_list;
    std::vector<Eigen::Vector3d> posi_loc_vec;
    std::vector<Eigen::Quaterniond> rot_loc_vec;
    std::vector<double> timestamp_list;
    //add by yao
    std::vector<Eigen::Vector3d> fea_match_vec;
    std::vector<raw_match>time_matchnum_vec;

};

}

