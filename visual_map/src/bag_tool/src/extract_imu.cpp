#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/stat.h>
#include <sys/types.h> 


int main(int argc, char **argv){

    std::string bag_addr=argv[1];
    std::string out_addr=argv[2];
    std::string imu_topic=argv[3];
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);    
    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int imu_count=0;
    rosbag::View::iterator it= view.begin();
    std::ofstream outfile;
    
    outfile.open (out_addr+"/imu.txt");
    if (!outfile.is_open())
    {
        std::cout<<"file not open"<<std::endl;
    }
    double sec_last=0;
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::ImuPtr simu = m.instantiate<sensor_msgs::Imu>();
        if(simu!=NULL){
            double sec = simu->header.stamp.toSec();
            std::stringstream ss;
            ss<<std::setprecision (15)<<imu_count<<","<<view.getBeginTime().toSec()<<","<<sec<<std::endl;
            if(sec- sec_last>0.02){
                std::cout<<"error: "<<imu_count<<":"<<sec- sec_last<<std::endl;
            }
            ss<<std::setprecision (15)<<sec<<","<<simu->angular_velocity.x<<","<<simu->angular_velocity.y<<","<<simu->angular_velocity.z<<","<<simu->linear_acceleration.x<<","<<simu->linear_acceleration.y<<","<<simu->linear_acceleration.z<<std::endl;
            outfile<<ss.str();
            imu_count++;
            sec_last=sec;
        }
    }
    outfile.close();
    return 0;
};
