#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "read_write_data_lib/read_write.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

int main(int argc, char* argv[]){
    std::string res_root=argv[1];
    std::string bag_addr=argv[2];
    std::string img_topic=argv[3];
    
    std::string image_config_addr=res_root+"/image_conf.txt";
    int width;
    int height; 
    float desc_scale;
    int desc_level; 
    int desc_count;
    CHAMO::read_image_info(image_config_addr, width, height, desc_scale, desc_level, desc_count);
    std::cout<<"image_conf: "<<width<<":"<<height<<std::endl;
    
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(img_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    
    std::vector<float> mvScaleFactor;
    mvScaleFactor.resize(desc_level);
    mvScaleFactor[0]=1.0f;
    for(int i=1; i<desc_level; i++){
        mvScaleFactor[i]=mvScaleFactor[i-1]*desc_scale;
    }
    std::vector<float> mvInvScaleFactor;
    mvInvScaleFactor.resize(desc_level);
    for(int i=0; i<desc_level; i++){
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
    }
    
    std::vector<std::vector<cv::Mat>> imgs;
    
    std::string kp_addr=res_root+"/kps.txt";
    std::vector<Eigen::Vector2f> kp_uvs;
    std::vector<std::string> kp_framename;
    std::vector<int> kp_octoves;
    CHAMO::read_kp_info(kp_addr, kp_uvs, kp_framename, kp_octoves);
    std::cout<<"kp_uvs: "<<kp_uvs.size()<<std::endl;
    
    std::map<std::string, int> imgname_frameid_map;
    
    
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            img_count++;
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                cv::Mat img= cv_ptr->image;
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                //std::cout<<"aaa"<<std::endl;
                std::stringstream ss_time;
                ss_time<<"img_"<<img_count<<".jpg";
                std::vector<cv::Mat> imgs_temp;
                imgs_temp.push_back(img.clone());
                for(int i=1; i<desc_level; i++){
                    cv::Mat resized_img;
                    float scale = mvInvScaleFactor[i];
                    cv::Size sz(cvRound((float)width*scale), cvRound((float)height*scale));
                    cv::resize(img, resized_img, sz, 0, 0, cv::INTER_LINEAR);
                    GaussianBlur(resized_img, resized_img, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);
                    imgs_temp.push_back(resized_img.clone());
                }
                imgs.push_back(imgs_temp);
                imgname_frameid_map[ss_time.str()]=imgs.size()-1;
                //std::cout<<ss_time.str()<<std::endl;
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    cv::Ptr<cv::DescriptorExtractor> extractor_ = cv::xfeatures2d::FREAK::create(false, true, 22, 8);
    std::vector<int> kf_id_list;
    for(int i=0; i<kp_uvs.size(); i++){
        int frameid = imgname_frameid_map[kp_framename[i]];
        int octove = kp_octoves[frameid];
        cv::KeyPoint kp;
        kp.pt.x=kp_uvs[frameid](0);
        kp.pt.y=kp_uvs[frameid](1);
        kp.octave=0;
        std::vector<cv::KeyPoint> keypoints;
        keypoints.push_back(kp);
        cv::Mat desc;
//         std::cout<<kp_uvs[frameid].transpose()<<std::endl;
//         std::cout<<imgs[frameid][octove].cols<<":"<<imgs[frameid][octove].rows<<std::endl;
        extractor_->compute(imgs[frameid][octove], keypoints, desc);
//         std::cout<<desc<<std::endl;
        if(keypoints.size()==0){
            std::cout<<"removed a kp!"<<std::endl;
        }
    }

    return 0;
}