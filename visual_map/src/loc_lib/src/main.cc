#include <loc_lib/ChamoLoc.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include "orb_slam_lib/two_frame_pose.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

std::unique_ptr<wayz::ChamoLoc> localizer;
double pose_time;
bool init_flag = false;

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    static double last_time =0.0;
    Eigen::Vector3d Accl(imu_msg->linear_acceleration.x,
                         imu_msg->linear_acceleration.y,
                         imu_msg->linear_acceleration.z);
    Eigen::Vector3d Gyro(imu_msg->angular_velocity.x,
                         imu_msg->angular_velocity.y,
                         imu_msg->angular_velocity.z);
    
    double timestamp = imu_msg->header.stamp.toSec();
    if(timestamp > last_time)
    {
        localizer->AddIMU(timestamp,Accl,Gyro);
        last_time = timestamp;
    }
}

void image_callback(const sensor_msgs::CompressedImageConstPtr& img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    cv::Mat img = cv_ptr->image;

    double timestamp = img_msg->header.stamp.toSec();
    if(!init_flag)
    {
        pose_time = img_msg->header.stamp.toSec()-0.1;
    }
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    localizer->AddImage(timestamp,0,img);
    init_flag = true;
}

void extract_bag(std::string bag_addr_, std::string img_topic, std::string imu_topic)
{
    std::string bag_addr = bag_addr_;
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(img_topic);
    topics.push_back(imu_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++)
    {
        rosbag::MessageInstance m = *it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL)
        {
            image_callback(simg);
        }

        sensor_msgs::ImuPtr simu = m.instantiate<sensor_msgs::Imu>();
        if(simu!=NULL)
        {
            imu_callback(simu);
        }
    }

}

void subscribe_bag(int argc, char* argv[],std::string img_topic, std::string imu_topic)
{
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1000);
    ros::Subscriber imu_subscriber_; 
    ros::Subscriber img_subscriber_;
    ros::Publisher  pose_pub;
    
    imu_subscriber_ = nh.subscribe("imu/raw_data", 1000, imu_callback);
    img_subscriber_ = nh.subscribe("camera/left/image_raw", 10000, image_callback);
    
    ros::start();
    Eigen::Vector3d Pos;
    Eigen::Vector3d Vel;
    Eigen::Quaterniond Ori;
    int counter = 0;
    while (ros::ok()) 
    {
//         ++counter;
//         if(init_flag && counter%10 == 0)
//         {
//             if(localizer->QueryPose(pose_time, Pos, Vel, Ori))
//             {
//                 counter = 0;
//             }
//             pose_time += 0.1;
//         }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}
                                        
int main(int argc, char* argv[])
{
    visualization::RVizVisualizationSink::init();
    std::string imgtopic = "img";
    std::string imutopic = "imu";
    std::string res_addr = argv[1];

    std::string bagfilepath;
    bool breadbag = false;
    if (argc > 2)
    {
        breadbag = true;
        bagfilepath = argv[2];
    }
    
    localizer.reset(new wayz::ChamoLoc);
    localizer->StartLocalization(res_addr+"/rovio_default_config.info");
    localizer->AddMap(res_addr);

    if (!breadbag)
    {
        subscribe_bag(argc,argv,imgtopic,imutopic);
    }
    else
    {
        extract_bag(bagfilepath,imgtopic,imutopic);
    }
    
    localizer->Export_Raw_MatchFile(res_addr);
    localizer->Shutdown();
    
    return 0;
}