#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "KeyFrameDatabase.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "ros/ros.h"
#include "global_match/orb_match.h"
#include "visual_map/visual_map_seri.h"
#include "read_write_data_lib/read_write.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace chamo {
    
    void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
        Eigen::Matrix3Xd points;
        points.resize(3,mp_posis.size());
        for(int i=0; i<mp_posis.size(); i++){
            points.block<3,1>(0,i)=mp_posis[i];
        }    
        publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
    }
        
    void show_pose_as_marker(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& rots, std::string topic){
        visualization::PoseVector poses_vis;
        for(int i=0; i<posis.size(); i=i+1){
            visualization::Pose pose;
            pose.G_p_B = posis[i];
            pose.G_q_B = rots[i];

            pose.id =poses_vis.size();
            pose.scale = 0.2;
            pose.line_width = 0.02;
            pose.alpha = 1;
            poses_vis.push_back(pose);
        }
        visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", topic);
    }

    void showMatchLine(std::vector<Eigen::Vector3d>& posi1, std::vector<Eigen::Vector3d>& posi2, Eigen::Vector3i color, float a, float w, std::string topic){
        visualization::LineSegmentVector matches;
        for(int i=0; i<posi1.size(); i++){
            visualization::LineSegment line_segment;
            line_segment.from = posi2[i];
            line_segment.scale = w;
            line_segment.alpha = a;
            line_segment.color.red = color(0);
            line_segment.color.green = color(1);
            line_segment.color.blue = color(2);
            line_segment.to = posi1[i];
            matches.push_back(line_segment);
        }
        visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, topic);
    }

    void findIdByName(std::vector<std::string>& names, int& re_id, std::string query_name){
        re_id=-1;
        for(int i=0; i<names.size(); i++){
            if(names[i]==query_name){
                re_id=i;
                return;
            }
        }
        return;
    }
    
    void visMap(ORB_SLAM2::Map* mpMap){
        std::vector<ORB_SLAM2::MapPoint*> mps_all=mpMap->GetAllMapPoints();
        std::vector<Eigen::Vector3d> mps_eig;
        for(int i=0; i<mps_all.size(); i++){
            cv::Mat center= mps_all[i]->GetWorldPos();
            Eigen::Vector3d center_eig;
            center_eig(0)=center.at<float>(0);
            center_eig(1)=center.at<float>(1);
            center_eig(2)=center.at<float>(2);
            mps_eig.push_back(center_eig);
        }
        show_mp_as_cloud(mps_eig, "temp_mp");

        std::vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        std::vector<Eigen::Vector3d> traj_kf;
        for(int i=0; i<vpKFs.size(); i++){
            cv::Mat center= vpKFs[i]->GetCameraCenter();
            Eigen::Vector3d center_eig;
            center_eig(0)=center.at<float>(0);
            center_eig(1)=center.at<float>(1);
            center_eig(2)=center.at<float>(2);
            traj_kf.push_back(center_eig);
        }
        show_mp_as_cloud(traj_kf, "temp_kf");
        if(0){
            std::vector<Eigen::Vector3d> kf_centers;
            std::vector<Eigen::Vector3d> mp_centers;
            for(int i=0; i<vpKFs.size(); i=i+100){
                cv::Mat center= vpKFs[i]->GetCameraCenter();
                Eigen::Vector3d center_eig;
                center_eig(0)=center.at<float>(0);
                center_eig(1)=center.at<float>(1);
                center_eig(2)=center.at<float>(2);
                std::vector<ORB_SLAM2::MapPoint*> kf_mps= vpKFs[i]->GetMapPointMatches();
                for(int j=0; j<kf_mps.size(); j++){
                    cv::Mat center_mp= kf_mps[j]->GetWorldPos();
                    Eigen::Vector3d center_mp_eig;
                    center_mp_eig(0)=center_mp.at<float>(0);
                    center_mp_eig(1)=center_mp.at<float>(1);
                    center_mp_eig(2)=center_mp.at<float>(2);
                    mp_centers.push_back(center_mp_eig);
                    kf_centers.push_back(center_eig);
                }
            }
            showMatchLine(mp_centers, kf_centers, Eigen::Vector3i(255,255,255),0.6, 0.03, "temp_match_mp");
            
            std::vector<Eigen::Vector3d> kf_centers1;
            std::vector<Eigen::Vector3d> kf_centers2;
            for(int i=0; i<vpKFs.size(); i=i+10){
                std::vector<ORB_SLAM2::KeyFrame*> kfs= vpKFs[i]->GetVectorCovisibleKeyFrames();
                cv::Mat center= vpKFs[i]->GetCameraCenter();
                Eigen::Vector3d center_eig;
                center_eig(0)=center.at<float>(0);
                center_eig(1)=center.at<float>(1);
                center_eig(2)=center.at<float>(2);
                for(int j=0; j<kfs.size(); j++){
                    cv::Mat center2 = kfs[j]->GetCameraCenter();
                    Eigen::Vector3d center2_eig;
                    center2_eig(0)=center2.at<float>(0);
                    center2_eig(1)=center2.at<float>(1);
                    center2_eig(2)=center2.at<float>(2);
                    kf_centers1.push_back(center_eig);
                    kf_centers2.push_back(center2_eig);
                }
            }
            showMatchLine(kf_centers1, kf_centers2, Eigen::Vector3i(255,255,0),1, 0.03, "temp_match_kf");
            
            kf_centers1.clear();
            kf_centers2.clear();
            for(int i=0; i<vpKFs.size(); i=i+1){
                ORB_SLAM2::KeyFrame* kf2= vpKFs[i]->GetParent();
                if(kf2!= NULL){
                    cv::Mat center= vpKFs[i]->GetCameraCenter();
                    Eigen::Vector3d center_eig;
                    center_eig(0)=center.at<float>(0);
                    center_eig(1)=center.at<float>(1);
                    center_eig(2)=center.at<float>(2);
                    cv::Mat center2 = kf2->GetCameraCenter();
                    Eigen::Vector3d center2_eig;
                    center2_eig(0)=center2.at<float>(0);
                    center2_eig(1)=center2.at<float>(1);
                    center2_eig(2)=center2.at<float>(2);
                    kf_centers1.push_back(center_eig);
                    kf_centers2.push_back(center2_eig);
                }
            }
            showMatchLine(kf_centers1, kf_centers2, Eigen::Vector3i(255,0,255),1, 0.3, "temp_span_kf");
        }
    }
    
    Eigen::Matrix4d MatchWithGlobalMap(ORB_SLAM2::Frame& frame, 
                                    ORB_SLAM2::ORBVocabulary* mpVocabulary, 
                                    ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase, 
                                    ORB_SLAM2::Map* mpMap){
        frame.ComputeBoW();
        std::vector<ORB_SLAM2::KeyFrame*> vpCandidateKFs = mpKeyFrameDatabase->DetectRelocalizationCandidates(&frame);
        std::cout<<"frame name: "<<frame.file_name_<<std::endl;
        for(int i=0; i<vpCandidateKFs.size(); i++){
            std::cout<<vpCandidateKFs[i]->file_name_<<std::endl;
        }

    }
    void printFrameInfo(ORB_SLAM2::Frame& frame){
        std::cout<<"kp count: "<<frame.mvKeysUn.size()<<std::endl;
        std::cout<<"desc size (w:h) "<<frame.mDescriptors.cols<<":"<<frame.mDescriptors.rows<<std::endl;
    }
    
    void GetAFrame(cv::Mat img, ORB_SLAM2::Frame& frame, ORB_SLAM2::ORBextractor* mpORBextractor, ORB_SLAM2::ORBVocabulary*& mpVocabulary,
                   cv::Mat mK, cv::Mat mDistCoef, std::string file_name, double timestamp){
        cv::Mat mImGray;
        cv::undistort(img, mImGray, mK, mDistCoef);
        cv::Mat distCoefZero=cv::Mat::zeros(mDistCoef.rows, mDistCoef.cols, mDistCoef.type());
        frame = ORB_SLAM2::Frame(mImGray, timestamp, mpORBextractor, mpVocabulary, mK, distCoefZero, 0, 0, file_name, false);
        //cv::imshow("sdfsd", mImGray);
        //cv::waitKey(1);
        //printFrameInfo(frame);
    }
    
    void GetORBextractor(std::string res_root, ORB_SLAM2::ORBextractor** mpORBextractor, cv::Mat& mK, cv::Mat& mDistCoef){
        std::string cam_addr=res_root+"/camera_config.txt";
        Eigen::Matrix3d cam_inter;
        Eigen::Vector4d cam_distort;
        Eigen::Matrix4d Tbc;
        CHAMO::read_cam_info(cam_addr, cam_inter, cam_distort, Tbc);
        std::cout<<"cam_inter: "<<cam_inter<<std::endl;
        std::string image_config_addr=res_root+"/image_conf.txt";
        int width;
        int height; 
        float desc_scale;
        int desc_level; 
        int desc_count;
        CHAMO::read_image_info(image_config_addr, width, height, desc_scale, desc_level, desc_count);
        std::cout<<"image_conf: (desc_scale: desc_level: desc_count)"<<desc_scale<<":"<<desc_level<<":"<<desc_count<<std::endl;
        *mpORBextractor = new ORB_SLAM2::ORBextractor(desc_count,desc_scale,desc_level,20,7);
        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = cam_inter(0,0);
        mK.at<float>(1,1) = cam_inter(1,1);
        mK.at<float>(0,2) = cam_inter(0,2);
        mK.at<float>(1,2) = cam_inter(1,2);
        mDistCoef = cv::Mat(1,4,CV_32F);
        mDistCoef.at<float>(0) = cam_distort(0);
        mDistCoef.at<float>(1) = cam_distort(1);
        mDistCoef.at<float>(2) = cam_distort(2);
        mDistCoef.at<float>(3) = cam_distort(3);
    }
    
    
    
}