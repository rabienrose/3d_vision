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

#include "VisualMap.pb.h"
#include "visual_map/visual_map.h"

void seri_img(proto::VisualMap& map, cv::Mat img, double timestamp, std::string filename) {
    proto::Frame* frame_p = map.add_frames();
    frame_p->set_img_name(filename);
    frame_p->set_timestamp(timestamp);
    
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;
    detector->detectAndCompute(img, cv::noArray(), keypoints1, descriptors1);
    //descriptors1.cols is the width of descriptors
    //descriptors1.rows is the number of descriptors
    
    for (int i=0; i<keypoints1.size(); i++){
        proto::KeyPoint* kp = frame_p->add_kps();
        kp->set_u(keypoints1[i].pt.x);
        kp->set_v(keypoints1[i].pt.y);
        kp->set_desc_length(descriptors1.cols);
        kp->set_desc_type(2);//1: float,   2: byte
        std::string temp_ss;
        for(int j=0; j<descriptors1.cols; j++){
            temp_ss.push_back(descriptors1.at<unsigned char>(j, i));
            //std::cout<<(int)descriptors1.at<unsigned char>(j, i)<<std::endl;
        }
        //std::cout<<temp_ss<<std::endl;
        kp->set_desc_byte(temp_ss);
    }
}

void save_map(proto::VisualMap& visual_map, std::string& save_addr){
    std::fstream output(save_addr.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
    
    if (!visual_map.SerializeToOstream(&output)) {
        std::cerr << "Failed to write map data." << std::endl;
        return;
    }
}

int main(int argc, char* argv[]) {
    std::string save_addr = argv[1];
    std::string bag_addr = argv[2];
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    
    std::cout<<bag_addr<<std::endl;
    
    std::vector<std::string> topics;
    topics.push_back("camera/right/image_raw");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    int img_count=100000;
    vm::VisualMap map;
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
        if(simg!=NULL){
            img_count++;
            if(img_count>100100){
                break;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                std::shared_ptr<vm::Frame> frame_p;
                frame_p.reset(new vm::Frame);
                double timestamp=simg->header.stamp.toSec(); 
                frame_p->time_stamp=timestamp;
                cv::Ptr<cv::ORB> detector = cv::ORB::create();
                std::vector<cv::KeyPoint> keypoints1;
                cv::Mat descriptors1;
                detector->detectAndCompute(cv_ptr->image, cv::noArray(), frame_p->kps, frame_p->descriptors);
                //descriptors1.cols is the width of descriptors
                //descriptors1.rows is the number of descriptors
                map.frames.push_back(frame_p);
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    
    for(int i=1; i<map.frames.size(); i++){
        std::vector<cv::DMatch> matches;
        cv::BFMatcher matcher;
        matcher.match(map.frames[i]->descriptors, map.frames[i-1]->descriptors, matches);
        std::sort(matches.begin(), matches.end());
        std::vector<cv::DMatch> good_matches;
        const int ptsPairs = std::min(1000, (int)(matches.size() * 0.2f));
        for( int j = 0; j < ptsPairs; j++ )
        {
            good_matches.push_back(matches[j]);
        }
        std::cout<<good_matches.size()<<std::endl;
    }
    
    return 0;
}