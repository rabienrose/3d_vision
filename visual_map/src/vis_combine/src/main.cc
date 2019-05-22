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
                                        
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    std::string bag_addr=argv[2];
    std::string img_topic=argv[3];
    
    ORB_SLAM2::ORBVocabulary* mpVocabulary;
    ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
    ORB_SLAM2::Map* mpMap;
    ORB_SLAM2::ORBextractor* mpORBextractor;

    std::vector<Eigen::Vector3d> re_traj;
    
    chamo::LoadORBMap(res_root, mpVocabulary, mpKeyFrameDatabase, mpMap);
    cv::Mat mK;
    cv::Mat mDistCoef;
    chamo::GetORBextractor(res_root, mpORBextractor, mK, mDistCoef);
    
    
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
                Eigen::Matrix4d re_pose = chamo::MatchWithGlobalMap(frame, mpVocabulary, mpKeyFrameDatabase, mpMap);
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    

    return 0;
}