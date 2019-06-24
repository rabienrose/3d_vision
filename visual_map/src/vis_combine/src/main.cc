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
#include "read_write_data_lib/read_write.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include"Frame.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include "global_match/orb_match.h"
#include "global_match/global_match.h"

DEFINE_string(map1_addr, "", "First map to merge.");
DEFINE_string(map2_addr, "", "Second map to merge.");
DEFINE_string(desc1_addr, "", "Descriptor index of first map to merge.");
DEFINE_string(desc2_addr, "", "Descriptor index of second map to merge.");
DEFINE_string(output_addr, "", "Address for the output map.");

              
void printFrameInfo(ORB_SLAM2::Frame& frame){
    std::cout<<"kp count: "<<frame.mvKeysUn.size()<<std::endl;
    std::cout<<"desc size (w:h) "<<frame.mDescriptors.cols<<":"<<frame.mDescriptors.rows<<std::endl;
}

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

void transformPoseUseSim3(Eigen::Matrix4d& sim3, double scale,  Eigen::Matrix4d& in_pose,  Eigen::Matrix4d& out_pose){
    Eigen::Matrix3d R_tran=sim3.block(0,0,3,3)/scale;
    Eigen::Matrix3d R_in=in_pose.block(0,0,3,3);
    Eigen::Matrix3d R_out=R_tran*R_in;
    Eigen::Vector4d t_out=sim3*in_pose.block(0,3,4,1);
    out_pose= Eigen::Matrix4d::Identity();
    out_pose.block(0,0,3,3) = R_out;
    out_pose.block(0,3,4,1) = t_out;
}

void convertToFrame(std::shared_ptr<vm::Frame> frame_vm, ORB_SLAM2::Frame& frame){
    int desc_width=frame_vm->descriptors.rows();
    int desc_count=frame_vm->descriptors.cols();
    frame.mDescriptors=cv::Mat(desc_count, desc_width , CV_8UC1);
    frame.mvKeysUn=frame_vm->kps;
    for(int i=0; i<desc_count; i++){
        for(int j=0; j<desc_width; j++){
            frame.mDescriptors.at<unsigned char>(i, j)=frame_vm->descriptors(j, i);
        }
    }
    frame.mDistCoef=cv::Mat::zeros(1,4,CV_32FC1);
    frame.mK=cv::Mat::eye(3,3,CV_32FC1);
    frame.mK.at<float>(0,0)=frame_vm->fx;
    frame.mK.at<float>(1,1)=frame_vm->fy;
    frame.mK.at<float>(0,2)=frame_vm->cx;
    frame.mK.at<float>(1,2)=frame_vm->cy;
}
            
int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    
    vm::VisualMap map1;
    vm::loader_visual_map(map1, FLAGS_map1_addr);
    vm::VisualMap map2;
    vm::loader_visual_map(map2, FLAGS_map2_addr);
    
    std::vector<Eigen::Vector3d> mp_posis1;
    map1.GetMPPosiList(mp_posis1);
    std::vector<Eigen::Vector3d> mp_posis2;
    map2.GetMPPosiList(mp_posis2);
    map1.ComputeUniqueId();
    map2.ComputeUniqueId();
    map1.CheckConsistence();
    map2.CheckConsistence();
    
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_1;
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_2;
    Eigen::MatrixXf projection_matrix_;
    chamo::LoadMap(FLAGS_desc1_addr, index_1, projection_matrix_);
    chamo::LoadMap(FLAGS_desc2_addr, index_2, projection_matrix_);
    
    for(int i=0; i<map1.frames.size(); i++){
        ORB_SLAM2::Frame frame;
        convertToFrame(map1.frames[i], frame);
        std::vector<int> inliers_mp;
        std::vector<int> inliers_kp;
        Eigen::Matrix4d pose;
        chamo::MatchImg(mp_posis2, index_2, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
        LOG(INFO)<<inliers_kp.size();
        for(int j=0; j<inliers_kp.size(); j++){
            CHECK_GT(map1.frames[i]->obss.size(), inliers_kp[j]);
            std::shared_ptr<vm::MapPoint> mp1=map1.frames[i]->obss[inliers_kp[j]];
            if(mp1!= nullptr){
                CHECK_GT(map2.mappoints.size(), inliers_mp[j]);
                std::shared_ptr<vm::MapPoint> mp2=map2.mappoints[inliers_mp[j]];
                CHECK_NOTNULL(mp2);
                if(mp2->id==-1){
                    continue;
                }
                if(mp2->id==mp1->id){
                    continue;
                }
                for(int k=0; k<mp2->track.size(); k++){
                    mp1->track.push_back(mp2->track[k]);
                    CHECK_GT(mp2->track[k].frame->obss.size(), mp2->track[k].kp_ind);
                    mp2->track[k].frame->obss[mp2->track[k].kp_ind]=mp1;
                    mp2->id=-1;
                }
            }
        }
    }
    
    //to-do due to duplicate mp issue, inverse match is not possible now
//     LOG(INFO)<<"finish match from map1 to map2";
//     //map1.CheckConsistence();
//     LOG(INFO)<<"start match from map2 to map1";
//     
//     for(int i=0; i<map2.frames.size(); i++){
//         ORB_SLAM2::Frame frame;
//         convertToFrame(map2.frames[i], frame);
//         std::vector<int> inliers_mp;
//         std::vector<int> inliers_kp;
//         Eigen::Matrix4d pose;
//         chamo::MatchImg(mp_posis1, index_1, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
//         LOG(INFO)<<inliers_kp.size();
//         for(int j=0; j<inliers_kp.size(); j++){
//             CHECK_GT(map2.frames[i]->obss.size(), inliers_kp[j]);
//             std::shared_ptr<vm::MapPoint> mp2=map2.frames[i]->obss[inliers_kp[j]];
//             //LOG(INFO)<<"mp2->id: "<<mp2->id;
//             if(mp2!= nullptr){
//                 CHECK_GT(map1.mappoints.size(), inliers_mp[j]);
//                 std::shared_ptr<vm::MapPoint> mp1=map1.mappoints[inliers_mp[j]];
//                 CHECK_NOTNULL(mp1);
//                 if(mp2->id==-1){
//                     continue;
//                 }
//                 if(mp2->id==mp1->id){
//                     continue;
//                 }
//                 for(int k=0; k<mp2->track.size(); k++){
//                     bool find_one=false;
//                     for(int n=0; n<mp1->track.size(); n++){
//                         if(mp1->track[n].frame->id==mp2->track[k].frame->id && mp1->track[n].kp_ind==mp2->track[k].kp_ind){
//                             find_one=true;
//                         }
//                     }
//                     if(find_one==true){
//                         //LOG(INFO)<<"Find dup one!!";
//                         continue;
//                     }
//                     mp1->track.push_back(mp2->track[k]);
//                     CHECK_GT(mp2->track[k].frame->obss.size(), mp2->track[k].kp_ind);
//                     mp2->track[k].frame->obss[mp2->track[k].kp_ind]=mp1;
//                     LOG(INFO)<<"mp2->id: "<<mp2->id;
//                     mp2->id=-1;
//                     LOG(INFO)<<"mp2->track[k].frame->id: "<<mp2->track[k].frame->id;
//                     LOG(INFO)<<"mp2->track[k].frame->obss[mp2->track[k].kp_ind]->id: "<<mp2->track[k].frame->obss[mp2->track[k].kp_ind]->id;
//                     LOG(INFO)<<"mp2->track[k].kp_ind: "<<mp2->track[k].kp_ind;
//                     LOG(INFO)<<"mp1->id: "<<mp1->id;
//                     
//                     map1.CheckConsistence();
//                 }
//                 
//             }
//         }
//     }
    
    for(int i=0; i<map2.frames.size(); i++){
        map2.frames[i]->position(0)=map2.frames[i]->position(0)+100;
        map2.frames[i]->position(1)=map2.frames[i]->position(1)+100;
        map1.frames.push_back(map2.frames[i]);
    }
    for(int i=0; i<map2.mappoints.size(); i++){
        if(map2.mappoints[i]->id!=-1){
            map2.mappoints[i]->position(0)=map2.mappoints[i]->position(0)+100;
            map2.mappoints[i]->position(1)=map2.mappoints[i]->position(1)+100;
            map1.mappoints.push_back(map2.mappoints[i]);
        }
    }
    map1.CheckConsistence();
    vm::save_visual_map(map1, FLAGS_output_addr);

    return 0;
}