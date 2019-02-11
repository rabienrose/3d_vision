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


void process_img(visual_map::VisualMap& map, cv::Mat img, double timestamp, std::string filename) {
    visual_map::Frame* frame_p = map.add_frames();
    frame_p->set_img_name(filename);
    frame_p->set_timestamp(timestamp);
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
    visual_map::VisualMap visual_map;
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
                std::stringstream ss;
                ss<<"img_"<<img_count<<std::endl;
                double timestamp=simg->header.stamp.toSec(); 
                process_img(visual_map, cv_ptr->image, timestamp, ss.str());
        
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    std::fstream output(save_addr.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
    
    if (!visual_map.SerializeToOstream(&output)) {
        std::cerr << "Failed to write map data." << std::endl;
        return -1;
    }
    return 0;
}