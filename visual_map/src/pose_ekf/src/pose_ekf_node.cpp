#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include "pose_ekf.h"
#include <algorithm>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "CoorConv.h"

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}

int main (int argc, char **argv) 
{
    ros::init(argc, argv, "loc_imu_gps");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_addr=argv[1];
    std::string bag_addr_=argv[2];
    std::string imu_topic=argv[4];
    std::string gps_topic=argv[5];
    std::string bag_addr=bag_addr_;
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(gps_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    bool is_init=false;
    rosbag::View::iterator it= view.begin();
    Eigen::Vector3d anchorGps;
    int gps_count=0;
    std::vector<Eigen::Vector3d> posi_vec;
    std::vector<Eigen::Vector3d> gps_vec;
    std::vector<double> gps_time;
    double updated_gps_id=0;
    bool first_gps_updated=false;
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::NavSatFixPtr sgps = m.instantiate<sensor_msgs::NavSatFix>();
        if(sgps!=NULL){
            if((int)sgps->position_covariance[0]<=10){
                Eigen::Vector3d coor_gps;
                Eigen::Vector3d ori_gps;
                ori_gps(0)=sgps->latitude;
                ori_gps(1)=sgps->longitude;
                ori_gps(2)=sgps->altitude;
                if(gps_count==0){
                    anchorGps=ori_gps;
                }
                gps_count++;
                double timestamp = sgps->header.stamp.toSec();
                convert_to_coor(ori_gps, coor_gps, anchorGps);
                gps_vec.push_back(coor_gps);
                gps_time.push_back(timestamp);
            }
        }       
    }
    show_mp_as_cloud(gps_vec, "temp_gps");
    it= view.begin();
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::ImuPtr simu = m.instantiate<sensor_msgs::Imu>();
        if(simu!=NULL){
            Eigen::Vector3d Accl(simu->linear_acceleration.x,
                         simu->linear_acceleration.y,
                         simu->linear_acceleration.z);
            Eigen::Vector3d Gyro(simu->angular_velocity.x,
                                simu->angular_velocity.y,
                                simu->angular_velocity.z);
        }
    }

  return 0;
}

