#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"System.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

using namespace std;

void show_pose_as_marker(std::vector<Eigen::Quaterniond>& rots, std::vector<Eigen::Vector3d>& posis, std::string topic){
    visualization::PoseVector poses_vis;
    for(int i=0; i<rots.size(); i=i+1){
        visualization::Pose pose;
        pose.G_p_B = posis[i];
        pose.G_q_B = rots[i];

        pose.id =poses_vis.size();
        pose.scale = 0.2;
        pose.line_width = 0.02;
        pose.alpha = 1;
        poses_vis.push_back(pose);
    }
    visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", topic);
}

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}

int main(int argc, char **argv)
{
    visualization::RVizVisualizationSink::init();
    ORB_SLAM2::System sys(argv[1],argv[2]);
    std::string bag_str=argv[3];
    std::string out_str=argv[4];
    rosbag::Bag bag;
    bag.open(bag_str,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    //topics.push_back("camera/right/image_raw");
    topics.push_back("img");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            cv_bridge::CvImagePtr cv_ptr;
            img_count++;
            if(img_count<0){
                continue;
            }
//             if(img_count >3000){
//                 break;
//             }
            try{
                
                std::stringstream ss;
                ss<<"img_"<<img_count<<".jpg";
                cv_ptr = cv_bridge::toCvCopy(simg, "mono8");
                sys.TrackMonocular(cv_ptr->image, simg->header.stamp.toSec(), ss.str());
                std::vector<Eigen::Vector3d> pcs;
                sys.getPC(pcs);
                std::vector<Eigen::Vector3d> posis;
                std::vector<Eigen::Quaterniond> quas;
                sys.getTraj(posis, quas);
                float reproject_err_t;
                int match_count_t;
                int mp_count_t;
                int kf_count_t;
                cv::Mat img_display;
                sys.getDebugImg(img_display, reproject_err_t, match_count_t, mp_count_t, kf_count_t);
                if(!img_display.empty()){
                    cv::imshow("chamo", img_display);
                    show_pose_as_marker(quas, posis, "temp_pose_loc");
                    cv::waitKey(1);
                }                
            }catch (cv_bridge::Exception& e){
                std::cout<<"err in main!!"<<std::endl;
                return 0;
            }
        }
    }
    sys.saveResult(out_str);
    return 0;
}
