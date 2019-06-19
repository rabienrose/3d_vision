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
#include "global_match/orb_match.h"
#include "global_match/global_match.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
              
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
            
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    std::string map_name=argv[2];
    std::string bag_name=argv[3];
    std::string img_topic=argv[4];
    std::string match_method=argv[5];
    
    vm::VisualMap map;
    vm::loader_visual_map(map, res_root+"/"+map_name);
    std::vector<Eigen::Vector3d> mp_posis;
    map->GetMPPosiList(mp_posis);
    
    ORB_SLAM2::ORBVocabulary* mpVocabulary;
    ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
    ORB_SLAM2::Map* mpMap;
    ORB_SLAM2::ORBextractor* mpORBextractor;
    
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_;
    Eigen::MatrixXf projection_matrix_;

    std::vector<Eigen::Vector3d> re_traj;
    
    if(match_method=="orb"){
        chamo::LoadORBMap(res_root, mpVocabulary, mpKeyFrameDatabase, mpMap);
    }else{
        chamo::LoadMap(res_root, index_, projection_matrix_);
    }
    
    cv::Mat mK;
    cv::Mat mDistCoef;
    chamo::GetORBextractor(res_root, &mpORBextractor, mK, mDistCoef);

    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(img_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    rosbag::View::iterator it= view.begin();
    ORB_SLAM2::Frame frame;
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            img_count++;
            if(img_count<0){
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
                
                std::vector<int> inliers_mp;
                std::vector<int> inliers_kp;
                Eigen::Matrix4d pose;
                if(match_method=="orb"){
                    chamo::MatchImg(mp_posis, index_, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
                }else{
                    pose = chamo::MatchWithGlobalMap(frame, mpVocabulary, mpKeyFrameDatabase, mpMap);
                }
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    return 0;
}