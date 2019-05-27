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
#include "vis_combine.h"
#include "global_match.h"
              
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
            
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root1=argv[1];
    std::string res_root2=argv[2];
    std::string bag_addr=argv[3];
    std::string img_topic=argv[4];
    
    ORB_SLAM2::ORBVocabulary* mpVocabulary;
    ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
    ORB_SLAM2::Map* mpMap;
    ORB_SLAM2::ORBextractor* mpORBextractor;

    std::vector<Eigen::Vector3d> re_traj;
    
    //chamo::LoadORBMap(res_root, mpVocabulary, mpKeyFrameDatabase, mpMap);
    cv::Mat mK;
    cv::Mat mDistCoef;
    chamo::GetORBextractor(res_root1, &mpORBextractor, mK, mDistCoef);
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_;
    Eigen::MatrixXf projection_matrix_;
    chamo::LoadMap(res_root1, index_, projection_matrix_);
    
    std::string posi_addr=res_root1+"/mp_posi_opt.txt";
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    std::cout<<"mp_posis: "<<mp_posis.size()<<std::endl;
    
    std::vector<Eigen::Matrix4d> poses_alin2;
    std::vector<std::string> frame_names2;
    std::string traj_file_addr2 = res_root2+"/frame_pose_opt.txt";
    CHAMO::read_traj_file(traj_file_addr2, poses_alin2, frame_names2);
    std::cout<<"frame_pose_opt2: "<<poses_alin2.size()<<std::endl;
    
    std::vector<Eigen::Matrix4d> poses_alin1;
    std::vector<std::string> frame_names1;
    std::string traj_file_addr1 = res_root1+"/frame_pose_opt.txt";
    CHAMO::read_traj_file(traj_file_addr1, poses_alin1, frame_names1);
    std::cout<<"frame_pose_opt1: "<<poses_alin1.size()<<std::endl;
    std::vector<Eigen::Vector3d> traj_posi1;
    for(int i=0; i<poses_alin1.size(); i++){
        traj_posi1.push_back(poses_alin1.block(0,3,3,1));
    }
    show_mp_as_cloud(traj_posi1, "traj_posi1");
    
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(img_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    rosbag::View::iterator it= view.begin();
    ORB_SLAM2::Frame frame;
    std::vector<Eigen::Vector3d> align_frame_posi2;
    std::vector<Eigen::Vector3d> frame_posi2;
    std::vector<int> frame_to_matched_id2;
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            img_count++;
            if(img_count<430){
                continue;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                cv::Mat img= cv_ptr->image;
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                std::stringstream ss_time;
                ss_time<<"img_"<<img_count<<".jpg";
                chamo::GetAFrame(img, frame, mpORBextractor, mpVocabulary, mK, mDistCoef, ss_time.str(), simg->header.stamp.toSec());
                //Eigen::Matrix4d re_pose = chamo::MatchWithGlobalMap(frame, mpVocabulary, mpKeyFrameDatabase, mpMap);
                std::vector<int> inliers_mp;
                std::vector<int> inliers_kp;
                Eigen::Matrix4d pose;
                chamo::MatchImg(mp_posis, index_, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
                if(inliers_kp.size()>0){
                    align_frame_posi2.push_back(pose.block(0,3,3,1));
                    frame_posi2.push_back(poses_alin2.block(0,3,3,1));
                    frame_to_matched_id2.push_back(align_img_posi2.size()-1);
                }else{
                    frame_to_matched_id2.push_back(-1);
                }
                
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    
    double scale_12;
    Eigen::Matrix4d T12;
    std::vector<Eigen::Matrix4d> transformed_traj2;
    orb_slam::ComputeSim3(align_frame_posi2, frame_posi2 , T12, scale_12);
    transformPoseUseSim3(T12, scale_12,  poses_alin2,  transformed_traj2);
    std::vector<Eigen::Vector3d> transformed_traj_posi2;
    for(int i=0; i<transformed_traj2.size(); i++){
        transformed_traj_posi2.push_back(transformed_traj2.block(0,3,3,1));
    }
    show_mp_as_cloud(transformed_traj_posi2, "transformed_traj_posi2");
    

    return 0;
}