#include <iostream>
#include <dirent.h>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/feature-track.h>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/matcher/match.h>
#include "aslam/matcher/match-visualization.h"
#include <aslam/matcher/matching-engine-exclusive.h>
#include <aslam/matcher/matching-problem-frame-to-frame.h>
#include "feature-tracking/feature-track-extractor.h"
#include "feature-tracking/feature-detection-extraction.h"
#include <inverted-multi-index/inverted-multi-index.h>
#include <maplab-common/binary-serialization.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "inverted-multi-index/inverted_multi_index.pb.h"
#include "two_frame_pose.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include "ORBmatcher.h"
#include "ORBextractor.h"

DEFINE_string(images_folder, "", "images ");
DEFINE_string(ncamera_calibration, "", "traj ");
DEFINE_int32(start_frame, 0, "traj ");
DEFINE_int32(end_frame, -1, "traj ");
DEFINE_string(quantizer_filename, "", "traj ");
DEFINE_string(index_addr, "", "traj ");
DEFINE_bool(generate_db, true, "traj ");
DEFINE_string(result_addr, "", "traj ");
DEFINE_int32(features_count, 2000, "traj ");
DEFINE_double(features_scale_rate, 1.2, "traj ");
DEFINE_int32(features_level, 8, "traj ");
DEFINE_int32(ini_cell_fast, 20, "traj ");
DEFINE_int32(min_cell_fast, 7, "traj ");

void convert_eigen_opencv_mat(const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& matrix, cv::Mat& mat){
    mat = cv::Mat(matrix.rows(), matrix.cols(), CV_8UC1);
    for(int i=0; i<matrix.rows(); i++){
        for(int j=0; j<matrix.cols(); j++){
            mat.at<unsigned char>(i, j)=matrix(i, j);
        }
    }
    mat=mat.t();
}

void convert_eigen_opencv_kp(const Eigen::Matrix2Xd& kps, std::vector<cv::KeyPoint>& kps_cv){
    for(int i=0; i<kps.cols(); i++){
        cv::KeyPoint key;
        key.pt.x=kps(0,i);
        key.pt.y=kps(1,i);
        kps_cv.push_back(key);
    }
}

bool FindRT(std::vector<cv::KeyPoint>& kp1, std::vector<cv::KeyPoint>& kp2, 
            cv::Mat& desc1, cv::Mat& desc2, int width, int height,
            std::vector<std::vector<std::vector<std::size_t>>>& mGrid,
            cv::Mat cam_m, Eigen::Matrix3d& R, Eigen::Vector3d& t, cv::Mat debug_img){
    std::vector<cv::Point2f> vbPrevMatched;
    vbPrevMatched.resize(kp1.size());
    for(size_t i=0; i<kp1.size(); i++)
        vbPrevMatched[i]=kp1[i].pt;
    std::vector<int> vnMatches12;
    orb_slam::SearchForInitialization(kp1, kp2, desc1, desc2, 0.9, mGrid, 0, 0, width, height, true, vbPrevMatched, vnMatches12);
    
    cv::Mat R21;
    cv::Mat t21;
    cv::Mat outlier_cv;
    std::vector<bool> vbMatchesInliers;
    std::vector<std::pair<int, int>> mvMatches12;
    for(size_t i=0, iend=vnMatches12.size();i<iend; i++)
    {
        if(vnMatches12[i]>=0)
        {
            mvMatches12.push_back(std::make_pair(i,vnMatches12[i]));
        }
    }
    std::cout<<"mvMatches12: "<<mvMatches12.size()<<std::endl;
    orb_slam::FindRT(kp1, kp2, vbMatchesInliers, R21, t21, mvMatches12 , cam_m, debug_img);
    
    if(R21.cols==0){
        return false;
    }else{
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                R(i,j)=R21.at<float>(i,j);
            }
        }
        for (int i=0; i<3; i++){
            t(i,0)=t21.at<float>(i,0);
        }
        return true;
    }
}

bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY, int mnMinX, int mnMinY, int mnMaxX, int mnMaxY)
{
    float mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
    float mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    std::cout<<FLAGS_images_folder<<std::endl;
    
    if (FLAGS_images_folder.empty()) {
        return -1;
    }

    int scale_rate=1;
    
    aslam::NCamera::Ptr camera_system = aslam::NCamera::loadFromYaml(FLAGS_ncamera_calibration);
    CHECK(camera_system) << "Could not load the camera calibration from: \'"<< FLAGS_ncamera_calibration << "\'";
    
    aslam::Camera::Ptr pinhole_A = camera_system->getCameraShared(0);
    
    aslam::PinholeCamera* pinhole = static_cast<aslam::PinholeCamera*>(pinhole_A.get());
    Eigen::Matrix3d cam_m_eigen = pinhole->getCameraMatrix();
    aslam::RadTanDistortion* distortion = static_cast<aslam::RadTanDistortion *>(pinhole_A->getDistortionMutable());
    Eigen::VectorXd dist_coef = distortion->getParameters();
    cv::Mat cam_m=cv::Mat::zeros(3, 3, CV_32FC1);
    cam_m.at<float>(0,0)=cam_m_eigen(0,0);
    cam_m.at<float>(1,1)=cam_m_eigen(1,1);
    cam_m.at<float>(0,2)=cam_m_eigen(0,2);
    cam_m.at<float>(1,2)=cam_m_eigen(1,2);
    cam_m.at<float>(2,2)=cam_m_eigen(2,2);
    cv::Mat cam_dis(4, 1, CV_32FC1);
    cam_dis.at<float>(0,0)=dist_coef(0,0);
    cam_dis.at<float>(1,0)=dist_coef(1,0);
    cam_dis.at<float>(2,0)=dist_coef(2,0);
    cam_dis.at<float>(3,0)=dist_coef(3,0);
    dist_coef(0,0)=0;
    dist_coef(1,0)=0;
    dist_coef(2,0)=0;
    dist_coef(3,0)=0;
    distortion->setParameters(dist_coef);
    
    std::unordered_map<aslam::FrameId, int> FrameId_to_index;
    float time_stamp=0;

    std::vector<std::shared_ptr<aslam::VisualNFrame>> frame_list;
    std::vector<std::vector<cv::KeyPoint>> kps_list;
    std::vector<cv::Mat> descriptors_list;
    std::vector<std::vector<std::vector<std::vector<std::size_t>>>> gird_list;
    std::vector<cv::Mat> img_list;
    int count_input=0;
    orb_slam::ORBextractor extractor = orb_slam::ORBextractor(FLAGS_features_count, FLAGS_features_scale_rate, FLAGS_features_level, 
                                   FLAGS_ini_cell_fast, FLAGS_min_cell_fast);
    for(int i=FLAGS_start_frame; i<FLAGS_end_frame ;i=i+1)
    {
        std::stringstream ss;
        ss<<"img_"<<i<<".jpg";
    
        std::string img_addr = FLAGS_images_folder + ss.str();
        cv::Mat img = cv::imread(img_addr);
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        //cv::resize(img,img,cv::Size(img.cols/scale_rate, img.rows/scale_rate));
        
        cv::Mat img_undistort;
        cv::undistort(img, img_undistort, cam_m, cam_dis);
        float width_img=img_undistort.cols;
        float height_img=img_undistort.rows;
        std::vector<cv::KeyPoint> keypoints;
        //std::cout<<width_img<<":"<<height_img<<std::endl;
        cv::Mat descriptors;
        extractor(img_undistort, keypoints, descriptors);
        //std::cout<<"keypoints: "<<keypoints.size()<<std::endl;
        kps_list.push_back(keypoints);
        descriptors_list.push_back(descriptors);
        std::vector<std::vector<std::vector<std::size_t>>> mGrid;
        int N=keypoints.size();
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        mGrid.resize(FRAME_GRID_COLS);
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++){
            mGrid[i].resize(FRAME_GRID_ROWS);
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
                mGrid[i][j].reserve(nReserve);
            }  
        }
        for(int i=0;i<N;i++)
        {
            const cv::KeyPoint &kp = keypoints[i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, 0, 0, width_img, height_img))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
        gird_list.push_back(mGrid);
        img_list.push_back(img_undistort);
        count_input++;
//         cv::Mat debug_img=img_undistort;
//         cv::cvtColor(debug_img, debug_img, cv::COLOR_GRAY2BGR);
//         for(int i=0; i<keypoints.size(); i++){
//             cv::circle(debug_img, keypoints[i].pt, 1, CV_RGB(0,0,255), 2);
//         }
//         cv::imshow("chamo", debug_img);
//         cv::waitKey(-1);
    }
    bool re =false;
    for (int i=0;i<kps_list.size();i=i+10){
        for(int j=i+4;j<kps_list.size();j++){
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            bool re = FindRT(kps_list[i], kps_list[j], descriptors_list[i], descriptors_list[j], 
                             img_list[i].cols ,img_list[i].rows ,gird_list[j] ,cam_m, R, t, img_list[i]);
            if (true){
                std::cout<<"succ in init: "<<std::endl;
                std::cout<<R<<std::endl;
                std::cout<<t<<std::endl;
                break;
            }
        }
        if (true){
            break;
        }
    }
    return 0;
}
