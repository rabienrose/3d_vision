#include <string>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "visual_map/visual_map.h"
#include "gtest/gtest.h"
#include <math.h>
#include "visual_map/visual_map_seri.h"

TEST(VisualMapTest, WriteAndLoad) {
    vm::VisualMap map;
    std::shared_ptr<vm::Frame> frame_p;
    frame_p.reset(new vm::Frame);
    frame_p->frame_file_name="chamo";
    frame_p->time_stamp=12000;
    cv::KeyPoint kp;
    kp.pt.x=200;
    kp.pt.y=300;
    frame_p->position.x()=2.2;
    frame_p->direction.w()=0.707;
    frame_p->kps.push_back(kp);
    frame_p->descriptors.resize(1,8);
    frame_p->descriptors(0,0)=128;
    frame_p->descriptors(0,7)=100;
    map.frames.push_back(frame_p);
    std::shared_ptr<vm::Frame> frame_p1;
    frame_p1.reset(new vm::Frame);
    frame_p1->frame_file_name="chamo";
    frame_p1->time_stamp=12001;
    frame_p1->position.y()=3.2;
    frame_p1->direction.z()=0.5;
    frame_p1->kps.push_back(kp);
    frame_p1->kps.push_back(kp);
    frame_p1->descriptors.resize(2,8);
    frame_p1->descriptors(0,2)=128;
    frame_p1->descriptors(0,4)=100;
    frame_p1->descriptors(1,4)=100;
    map.frames.push_back(frame_p1);
    
    std::shared_ptr<vm::MapPoint> mappoint_p;
    mappoint_p.reset(new vm::MapPoint);
    mappoint_p->position.x()=22;
    vm::TrackItem track_item;
    track_item.frame=frame_p;
    track_item.kp_ind=0;
    mappoint_p->track.push_back(track_item);
    track_item.frame=frame_p1;
    track_item.kp_ind=1;
    mappoint_p->track.push_back(track_item);
    map.mappoints.push_back(mappoint_p);
    
    frame_p->obss.push_back(mappoint_p);
    frame_p1->obss.push_back(mappoint_p);
    
    vm::save_visual_map(map, "chamo.bin");
    
    vm::VisualMap map_out;
    
    vm::loader_visual_map(map_out, "chamo.bin");
    
    std::cout<<map_out.frames.size()<<std::endl;
    std::cout<<map_out.mappoints.size()<<std::endl;
    
}

TEST(VisualMapTest, FillFrame) { 
//     std::string save_addr;
//     std::string bag_addr;
//     rosbag::Bag bag;
//     bag.open(bag_addr,rosbag::bagmode::Read);
//     
//     std::cout<<bag_addr<<std::endl;
//     
//     std::vector<std::string> topics;
//     topics.push_back("camera/right/image_raw");
//     rosbag::View view(bag, rosbag::TopicQuery(topics));
//     rosbag::View::iterator it= view.begin();
//     int img_count=100000;
//     vm::VisualMap map;
//     for(;it!=view.end();it++){
//         rosbag::MessageInstance m =*it;
//         sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
//         if(simg!=NULL){
//             img_count++;
//             if(img_count>100100){
//                 break;
//             }
//             cv_bridge::CvImagePtr cv_ptr;
//             try{
//                 cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
//                 std::shared_ptr<vm::Frame> frame_p;
//                 frame_p.reset(new vm::Frame);
//                 double timestamp=simg->header.stamp.toSec(); 
//                 frame_p->time_stamp=timestamp;
//                 cv::Ptr<cv::ORB> detector = cv::ORB::create();
//                 std::vector<cv::KeyPoint> keypoints1;
//                 cv::Mat descriptors1;
//                 detector->detectAndCompute(cv_ptr->image, cv::noArray(), frame_p->kps, frame_p->descriptors);
//                 //descriptors1.cols is the width of descriptors
//                 //descriptors1.rows is the number of descriptors
//                 map.frames.push_back(frame_p);
//             }catch (cv_bridge::Exception& e){
//                 ROS_ERROR("cv_bridge exception: %s", e.what());
//                 return;
//             }
//         }
//     }
//     
//     for(int i=1; i<map.frames.size(); i++){
//         std::vector<cv::DMatch> matches;
//         cv::BFMatcher matcher;
//         matcher.match(map.frames[i]->descriptors, map.frames[i-1]->descriptors, matches);
//         std::sort(matches.begin(), matches.end());
//         std::vector<cv::DMatch> good_matches;
//         const int ptsPairs = std::min(1000, (int)(matches.size() * 0.2f));
//         for( int j = 0; j < ptsPairs; j++ )
//         {
//             good_matches.push_back(matches[j]);
//         }
//     }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}