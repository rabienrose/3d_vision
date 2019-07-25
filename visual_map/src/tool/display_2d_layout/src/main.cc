#include <iostream>
#include <fstream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<Eigen/Core>
cv::Mat img1;
Eigen::Matrix4d trans;
void posi_callback(const geometry_msgs::Vector3ConstPtr& posi_msg){
    std::cout<<"get a posi"<<std::endl;
    Eigen::Vector4d homo_posi(posi_msg->x, posi_msg->y, posi_msg->z, 1);
    Eigen::Vector4d homo_map_posi=trans*homo_posi;
    std::cout<<homo_posi.transpose()<<std::endl;
    std::cout<<homo_map_posi.transpose()<<std::endl;
    cv::circle(img1, cv::Point2f(homo_map_posi(0), homo_map_posi(1)), 2, CV_RGB(255,0,0),3);
}

DEFINE_string(map_image_addr, "", "");
        
int main(int argc, char* argv[]){
    
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "show_2d_map");
    ros::NodeHandle nh;
    
    
    std::string layout_img_addr=FLAGS_map_image_addr;
    img1 = cv::imread(layout_img_addr);
    trans=Eigen::Matrix4d::Identity();
    trans(0,0)=-17.2981;
    trans(0,1)=0.347506;
    trans(0,2)=0;
    trans(0,3)=154.984;
    trans(1,0)=-0.347506;
    trans(1,1)=-17.2981;
    trans(1,2)=0;
    trans(1,3)=147.446;
    trans(2,0)=0;
    trans(2,1)=0;
    trans(2,2)=17.3016;
    trans(2,3)=0;
    
    std::cout<<trans<<std::endl;


    ros::Rate loop_rate(1000);
    ros::Subscriber point_subscriber_; 

    point_subscriber_ = nh.subscribe("loc_posi", 1000, posi_callback);
    while (ros::ok()) {
        cv::imshow("chamo", img1);
        cv::waitKey(100);
        ros::spinOnce();
        //loop_rate.sleep();
    }

    return 0;
}