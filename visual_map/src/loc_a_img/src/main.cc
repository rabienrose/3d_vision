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
#include <ctime>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <iostream>
#include <functional>
#include <sys/stat.h>


DEFINE_string(res_root, "", "");
DEFINE_string(map_name, "", "");
DEFINE_string(bag_addr, "", "");
DEFINE_string(match_method, "", "");


bool file_exists(const char *filename)
{
    struct stat file_stat;
    if (stat(filename, &file_stat) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void wait_user_input_path(std::function<bool(const std::string&)> on_user_input){
    bool finish = false;
    std::string user_input;
    while (!finish) {
    std::cout<<std::endl<<"input a file name, or end to quit:"<<std::endl;
    std::cin>>user_input;
        if(user_input == "end") {
            finish = true;
        } else {
            finish = on_user_input(user_input);
        }
    }
}
         
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    ros::Publisher posi_pub = nh.advertise<geometry_msgs::Vector3>("loc_posi", 1000, true);
    std::string res_root=FLAGS_res_root;
    std::string map_name=FLAGS_map_name;
    std::string bag_addr=FLAGS_bag_addr;
    std::string match_method=FLAGS_match_method;
    
    vm::VisualMap map;
    vm::loader_visual_map(map, map_name);
    std::vector<Eigen::Vector3d> mp_posis;
    map.GetMPPosiList(mp_posis);
    
    
    ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
    ORB_SLAM2::Map* mpMap;
    ORB_SLAM2::ORBextractor* mpORBextractor;
    
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_;
    Eigen::MatrixXf projection_matrix_;

    std::vector<Eigen::Vector3d> re_traj;
    
    if(match_method=="orb"){
        //chamo::LoadORBMap(res_root,map_name, mpVocabulary, mpKeyFrameDatabase, mpMap);
    }else{
        chamo::LoadMap(res_root, index_, projection_matrix_);
    }
    
    cv::Mat mK;
    cv::Mat mDistCoef;
    chamo::GetORBextractor(res_root, &mpORBextractor, mK, mDistCoef);

    std::string bash_dir = FLAGS_bag_addr;

    wait_user_input_path([&bash_dir, &index_, &projection_matrix_, &mp_posis, &posi_pub, &mpORBextractor, &mK, &mDistCoef](const std::string& file_name){
        std::string full_file_path = bash_dir + file_name;
        ORB_SLAM2::ORBVocabulary* mpVocabulary;
        if(file_exists(full_file_path.c_str())) {
            ORB_SLAM2::Frame frame;
            std::cout<<"Get user input path: "<< full_file_path << std::endl;
            cv::Mat img = cv::imread(full_file_path);
            cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
            std::stringstream ss_time;
            ss_time<<"img_"<<0<<".jpg";
            chamo::GetAFrame(img, frame, mpORBextractor, mpVocabulary, mK, mDistCoef, ss_time.str(), 0);
            std::vector<int> inliers_mp;
            std::vector<int> inliers_kp;
            Eigen::Matrix4d pose;
            if(false){
                //pose = chamo::MatchWithGlobalMap(frame, mpVocabulary, mpKeyFrameDatabase, mpMap);
            }else{
                chamo::MatchImg(mp_posis, index_, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
            }
            if(inliers_kp.size()>20){
                
                geometry_msgs::Vector3 posi_msg;
                posi_msg.x=pose(0,3);
                posi_msg.y=pose(1,3);
                posi_msg.z=pose(2,3);
                posi_pub.publish(posi_msg);
                std::cout<<"Result: "<<pose.block(0,3,3,1).transpose()<<std::endl;
            }else{
                std::cout<<"Fail to loc!!"<<std::endl;
            }
            
        } else {
            std::cout<<"Get user invalid input path: "<< full_file_path << std::endl;
        }
        return false;
    });
    return 0;
}