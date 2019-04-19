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


int main(int argc, char **argv){

    std::string bag_addr=argv[1];
    std::string out_dir=argv[2];
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);

    int camera_count = 1;
    
    
    while (argc - camera_count - 2 != 0)
    {
        std::string camera_topic = argv[camera_count+2];        
        std::vector<std::string> topics;
        topics.push_back(camera_topic);
        std::cout<<camera_topic<<std::endl;
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        int img_count=0;
        rosbag::View::iterator it= view.begin();
        std::ofstream outfile;
        
        outfile.open (out_dir+"/image_time.txt");
        if (!outfile.is_open())
        {
            std::cout<<"file not open"<<std::endl;
        }
        std::string dir_name = out_dir+"/camera_"+std::to_string(camera_count)+"_img";
        //_mkdir(dir_name.c_str(), NULL);
        int isCreate = mkdir(dir_name.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        
        for(;it!=view.end();it++){
            
            rosbag::MessageInstance m =*it;

            sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
            
            if(simg!=NULL){
                
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                    std::stringstream ss;
                    std::stringstream ss_time;
                    ss<<out_dir+"/camera_"+std::to_string(camera_count)+"_img/img_"<<img_count<<".jpg";

                    //cv::imwrite(ss.str(), cv_ptr->image);
                    ss_time<<"img_"<<img_count<<".jpg"<<","<<simg->header.stamp<<std::endl;
                    outfile<<ss_time.str();
            
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return 0;
                }
                img_count++;
            }
            
            sensor_msgs::CompressedImagePtr simgc = m.instantiate<sensor_msgs::CompressedImage>();
            
            if(simgc!=NULL){
                
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(simgc, "bgr8");
                    std::stringstream ss;
                    std::stringstream ss_time;
                    ss<<out_dir+"/camera_"+std::to_string(camera_count)+"_img/img_"<<img_count<<".jpg";

                    cv::imwrite(ss.str(), cv_ptr->image);
                    ss_time<<"img_"<<img_count<<".jpg"<<","<<simgc->header.stamp<<std::endl;
                    outfile<<ss_time.str();
            
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return 0;
                }
                img_count++;
            }
        }
        outfile.close();
        
        ++camera_count;
    }
    

    
    return 0;
};
