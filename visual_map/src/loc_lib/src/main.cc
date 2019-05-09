#include <loc_lib/ChamoLoc.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

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

                                        
int main(int argc, char* argv[]){
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    localizer.reset(new wayz::ChamoLoc);
    std::string res_addr=argv[1];
    localizer->StartLocalization(res_addr+"/rovio_default_config.info"); 
    localizer->AddMap(res_addr);
    
    ros::Rate loop_rate(1000);
    ros::Subscriber imu_subscriber_; 
    ros::Subscriber img_subscriber_;
    ros::Publisher  pose_pub;
    
    imu_subscriber_ = nh.subscribe("imu/raw_data", 1000, imu_callback);
    img_subscriber_ = nh.subscribe("camera/right/image_raw", 1, image_callback);
    //imu_subscriber_ = nh.subscribe("imu", 1000, imu_callback);
    //img_subscriber_ = nh.subscribe("img", 10, image_callback);
    
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
    localizer->Shutdown();
    ros::shutdown();
    

    return 0;
}