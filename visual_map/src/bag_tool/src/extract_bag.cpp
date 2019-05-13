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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <bag_tool/extract_bag.h>
#include "CoorConv.h"
#include <Eigen/Core>


void extract_bag(std::string out_addr_, std::string bag_addr_, std::string img_topic, std::string imu_topic, std::string gps_topic){

    std::string bag_addr=bag_addr_;
    std::string out_dir=out_addr_;
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(img_topic);
    topics.push_back(imu_topic);
    topics.push_back(gps_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it= view.begin();
    std::ofstream outfile_img_time;
    outfile_img_time.open (out_dir+"/image_time.txt");
    
    std::ofstream outfile_imu;
    outfile_imu.open (out_dir+"/imu.txt");
    
    std::ofstream outfile_gps;
    outfile_gps.open (out_dir+"/gps.txt");
    
    std::ofstream outfile_gps_orth;
    outfile_gps_orth.open (out_dir+"/gps_orth.txt");
    int gps_count=0;
    Eigen::Vector3d anchorGps;
    for(;it!=view.end();it++){
        
        rosbag::MessageInstance m =*it;

        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                std::stringstream ss;
                ss<<out_dir+"/images/img_"<<img_count<<".jpg";
                cv::imwrite(ss.str(), cv_ptr->image);
                std::stringstream ss_time;
                ss_time<<"img_"<<img_count<<".jpg"<<","<<simg->header.stamp<<std::endl;
                outfile_img_time<<ss_time.str();
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            img_count++;
        }
        
        sensor_msgs::ImuPtr simu = m.instantiate<sensor_msgs::Imu>();
        if(simu!=NULL){
            double sec = simu->header.stamp.toSec();
            std::stringstream ss;
            ss<<std::setprecision (15)<<sec<<","<<simu->angular_velocity.x<<","<<simu->angular_velocity.y<<","<<simu->angular_velocity.z<<","<<simu->linear_acceleration.x<<","<<simu->linear_acceleration.y<<","<<simu->linear_acceleration.z<<std::endl;
            outfile_imu<<ss.str();
        }
        
        sensor_msgs::NavSatFixPtr sgps = m.instantiate<sensor_msgs::NavSatFix>();
        if(sgps!=NULL){
            double sec = sgps->header.stamp.toSec();
            std::stringstream ss;
            ss<<std::setprecision (15)<<sec<<","<<sgps->latitude<<","<<sgps->longitude<<","<<sgps->altitude<<","<<(int)sgps->position_covariance[0]<<std::endl;
            outfile_gps<<ss.str();
            Eigen::Vector3d coor_gps;
            Eigen::Vector3d ori_gps;
            ori_gps(0)=sgps->latitude;
            ori_gps(1)=sgps->longitude;
            ori_gps(2)=sgps->altitude;
            if(gps_count==0){
                anchorGps=ori_gps;
                std::stringstream ss1;
                ss1<<std::setprecision (15)<<anchorGps(0)<<","<<anchorGps(1)<<","<<anchorGps(2)<<std::endl; 
                outfile_gps_orth<<ss1.str();
            }
            gps_count++;
            convert_to_coor(ori_gps, coor_gps, anchorGps);
            std::stringstream ss1;
            ss1<<std::setprecision (15)<<sec<<","<<coor_gps(0)<<","<<coor_gps(1)<<","<<coor_gps(2)<<","<<(int)sgps->position_covariance[0]<<std::endl; 
            outfile_gps_orth<<ss1.str();
        }
    }
    outfile_img_time.close();
    outfile_imu.close();
    outfile_gps.close();
    outfile_gps_orth.close();
};
