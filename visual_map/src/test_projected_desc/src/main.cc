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
#include <opencv2/core/eigen.hpp>
#include <random>
#include <math.h>
#include "descriptor-projection/build-projection-matrix.h"


void get_eigen_desc(cv::Mat descs, int index, Eigen::MatrixXf& desc_eigen ){
    cv2eigen(descs.row(index).t(), desc_eigen);
}

void append_eigen_desc(Eigen::MatrixXf& desc_eigen, Eigen::MatrixXf& new_desc_eigen ){
    int old_count=desc_eigen.cols();
    desc_eigen.conservativeResize(new_desc_eigen.rows(), old_count+new_desc_eigen.cols());
    desc_eigen.block(0, old_count, new_desc_eigen.rows(), new_desc_eigen.cols())=new_desc_eigen;
}

int find_in_last_matches(std::vector<cv::DMatch>& last_matches, int local_id ){
    for (int i=0; i<last_matches.size(); i++){
        if (last_matches[i].trainIdx==local_id){
            return 1;
        }
    }
    return -1;
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    std::string save_addr;
    std::string bag_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/wayz_2018_11_26.bag";
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    
    std::cout<<bag_addr<<std::endl;
    
    std::vector<std::string> topics;
    topics.push_back("camera/right/image_raw");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    int img_count=100000;
    int max_frame_count=10;
    int start_frame=0;
    cv::Mat last_descriptors;
    std::vector<cv::KeyPoint> last_keypoints;
    std::vector<cv::DMatch> last_matches;
    
    std::vector<std::vector<int>> tracks;
    
    std::unordered_map<int, int> last_local_id_to_global_id; //after combine all the descriptors together, 
    //the id of descriptor is new, we call this global id, respect with the id defined for each frame (local id)
    std::unordered_map<int, int> global_id_to_track_id; //recorded each desc id is belonged to which track
    std::vector<std::pair<int, int>> kp_infos;
    Eigen::MatrixXf global_descs;
    
    std::vector<cv::Mat> imgs;
    std::vector<std::vector<cv::KeyPoint>> kps;
    
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
        if(simg!=NULL){
            img_count++;
            if(img_count <start_frame || img_count>100000+max_frame_count){
                break;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "mono8");
                cv::Ptr<cv::ORB> detector = cv::ORB::create();
                std::vector<cv::KeyPoint> keypoints;
                cv::Mat descriptors;
                imgs.push_back(cv_ptr->image);
                
                detector->detectAndCompute(cv_ptr->image, cv::noArray(), keypoints, descriptors);
                kps.push_back(keypoints);
                if(last_keypoints.size()==0){
                    last_keypoints = keypoints;
                    last_descriptors=descriptors;
                    continue;
                }
                cv::BFMatcher matcher;
                std::vector<cv::DMatch> matches;
                matcher.match(last_descriptors, descriptors, matches); //descriptors.cols is the width of descriptor
                std::sort(matches.begin(), matches.end());
                std::vector<cv::DMatch> good_matches;
                const int ptsPairs = std::min(1000, (int)(matches.size() * 0.6f));
                for( int j = 0; j < ptsPairs; j++ )
                {
                    good_matches.push_back(matches[j]);
                }
                std::unordered_map<int, int> last_local_id_to_global_id_temp;
                for(int j=0; j<good_matches.size(); j++){
                    int last_local_id=good_matches[j].queryIdx;
                    int cur_local_id=good_matches[j].trainIdx;
                    Eigen::MatrixXf cur_desc_eigen;
                    get_eigen_desc(descriptors, cur_local_id, cur_desc_eigen);
                    append_eigen_desc(global_descs, cur_desc_eigen);
                    kp_infos.push_back(std::make_pair(imgs.size()-1, cur_local_id));
                    int cur_global_id=global_descs.cols()-1;
                    last_local_id_to_global_id_temp[cur_local_id]=cur_global_id;
                    
                    int re = find_in_last_matches(last_matches, last_local_id);
                    if(re>0){
                        int last_global_id;
                        if(last_local_id_to_global_id.count(last_local_id)!=0){
                            last_global_id=last_local_id_to_global_id[last_local_id];
                        }else{
                            std::cout<<"last_local_id not exists in last_local_id_to_global_id"<<std::endl;
                        }
                        int track_id;
                        if(global_id_to_track_id.count(last_global_id)!=0){
                            track_id = global_id_to_track_id[last_global_id];
                        }else{
                            std::cout<<"last_global_id not exists in global_id_to_track_id"<<std::endl;
                        }
                        tracks[track_id].push_back(cur_global_id);
                        global_id_to_track_id[cur_global_id]=track_id;
                    }else{
                        Eigen::MatrixXf last_desc_eigen;
                        get_eigen_desc(last_descriptors, last_local_id, last_desc_eigen);
                        append_eigen_desc(global_descs, last_desc_eigen);
                        kp_infos.push_back(std::make_pair(imgs.size()-2, last_local_id));
                        int last_global_id=global_descs.cols()-1;
                        std::vector<int> new_track;
                        new_track.push_back(last_global_id);
                        new_track.push_back(cur_global_id);
                        tracks.push_back(new_track);
                        global_id_to_track_id[last_global_id]=tracks.size()-1;
                        global_id_to_track_id[cur_global_id]=tracks.size()-1;
                    }
                }
                last_keypoints = keypoints;
                last_descriptors=descriptors;
                last_matches=good_matches;
                last_local_id_to_global_id=last_local_id_to_global_id_temp;
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    cv::RNG rng(12345);
    for (int i=0; i<tracks.size(); i++){
        std::vector<int> track=tracks[i];
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
        for (size_t j = 0; j < track.size(); ++j) {
            int global_id = track[j];
            int frame_id= kp_infos[global_id].first;
            int kp_id= kp_infos[global_id].second;
            cv::Mat img= imgs[frame_id];
            Eigen::MatrixXf desc=global_descs.col(global_id).transpose();
            
            std::vector<cv::KeyPoint>& kp=kps[frame_id];
            std::cout<<desc<<std::endl;
            cv::cvtColor(img,img, CV_GRAY2BGRA);
            cv::circle(img, kp[kp_id].pt, 2, color,2);
            cv::imshow("chamo", img);
            cv::waitKey(-1);
        }
    }
}