#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "KeyFrameDatabase.h"
#include "read_write_data_lib/read_write.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "global_match/orb_match.h"
#include "global_match/global_match.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"

#include <ctime>
struct raw_match
{
    double timestamp;
    int gmatchnum;
    double runtime;
};
              
void printFrameInfo(ORB_SLAM2::Frame& frame){
    std::cout<<"kp count: "<<frame.mvKeysUn.size()<<std::endl;
    std::cout<<"desc size (w:h) "<<frame.mDescriptors.cols<<":"<<frame.mDescriptors.rows<<std::endl;
}

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}
    
void show_pose_as_marker(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& rots, std::string topic){
    visualization::PoseVector poses_vis;
    for(int i=0; i<posis.size(); i=i+1){
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

void Export_Raw_MatchFile(std::string& path, std::vector<raw_match>& time_matchnum_vec)
{
    std::ofstream outfile_raw;
    outfile_raw.open(path + "/raw_match.txt");
    std::vector<raw_match>::iterator time_matchnum_itor = time_matchnum_vec.begin();
    for(;time_matchnum_itor != time_matchnum_vec.end();time_matchnum_itor++)
    {
        std::stringstream raw_info;
        raw_info << std::setprecision(16) << time_matchnum_itor->timestamp << "," ;
        raw_info << time_matchnum_itor->gmatchnum << ",";
        raw_info << time_matchnum_itor->runtime << std::endl;
        outfile_raw << raw_info.str();
    }
    
    outfile_raw.close();
}
            
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    std::string map_name=argv[2];
    std::string bag_addr=argv[3];
    std::string img_topic=argv[4];
    std::string match_method=argv[5];
    
    vm::VisualMap map;
    vm::loader_visual_map(map, map_name);
    std::vector<Eigen::Vector3d> mp_posis;
    map.GetMPPosiList(mp_posis);
    show_mp_as_cloud(mp_posis, "global_match_test_mp");
    
    ORB_SLAM2::ORBVocabulary* mpVocabulary;
    ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
    ORB_SLAM2::Map* mpMap;
    ORB_SLAM2::ORBextractor* mpORBextractor;
    
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_;
    Eigen::MatrixXf projection_matrix_;

    std::vector<Eigen::Vector3d> re_traj;
    
    if(match_method=="orb"){
        //chamo::LoadORBMap(res_root,map_name, mpVocabulary, mpKeyFrameDatabase, mpMap);
    }else{
        chamo::LoadMap(res_root, index_, projection_matrix_);
    }
    
    cv::Mat mK;
    cv::Mat mDistCoef;
    chamo::GetORBextractor(res_root, &mpORBextractor, mK, mDistCoef);

    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(img_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    rosbag::View::iterator it= view.begin();
    ORB_SLAM2::Frame frame;
    std::vector<Eigen::Vector3d> re_posis;
    std::vector<raw_match> time_matchnum_vec;
    ofstream f;
    f.open(res_root+"/frame_pose_opt.txt");
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            img_count++;
            if(img_count<0){
                continue;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try{
                
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                cv::Mat img= cv_ptr->image;
                double timestamp = simg->header.stamp.toSec();
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                std::stringstream ss_time;
                ss_time<<"img_"<<img_count<<".jpg";
                chamo::GetAFrame(img, frame, mpORBextractor, mpVocabulary, mK, mDistCoef, ss_time.str(), simg->header.stamp.toSec());
                
                std::vector<int> inliers_mp;
                std::vector<int> inliers_kp;
                Eigen::Matrix4d pose;
                clock_t start = clock();
                if(match_method=="orb"){
                    pose = chamo::MatchWithGlobalMap(frame, mpVocabulary, mpKeyFrameDatabase, mpMap);
                }else{
                    chamo::MatchImg(mp_posis, index_, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
                }
                
                double dur = (double)(clock() - start)/CLOCKS_PER_SEC;
                raw_match su = {0};
                su.runtime = dur;
                su.timestamp = timestamp;
                su.gmatchnum = inliers_mp.size();
                time_matchnum_vec.push_back(su);
                if(inliers_mp.size()>=20){
                    re_posis.push_back(pose.block(0,3,3,1));
                    std::stringstream ss;
                    f<<ss_time.str()<<",";

                    f << timestamp<<","<< pose(0,0) << "," << pose(0,1)  << "," << pose(0,2) << ","  << pose(0,3) << "," <<
                        pose(1,0) << "," << pose(1,1)  << "," << pose(1,2) << ","  << pose(1,3) << "," <<
                        pose(2,0) << "," << pose(2,1)  << "," << pose(2,2) << ","  << pose(2,3) << std::endl;
                    
                }
                if(img_count%5==0){
                    show_mp_as_cloud(re_posis, "global_match_test_traj");
                }
                cv::Mat debug_img;
                cv::cvtColor(img, debug_img, cv::COLOR_GRAY2RGB);
                for(int i=0; i<inliers_kp.size(); i++){
                    cv::circle(debug_img, frame.mvKeysUn[inliers_kp[i]].pt, 4, CV_RGB(0,0,255), 2);
                }
                visualization::RVizVisualizationSink::publish("global_match_test_img", debug_img);
                if(inliers_mp.size()>=20){
                    visualization::LineSegmentVector matches;
                    for(int i=0; i<inliers_mp.size(); i++){
                        visualization::LineSegment line_segment;
                        line_segment.from = re_posis.back();
                        line_segment.scale = 0.03;
                        line_segment.alpha = 0.6;

                        line_segment.color.red = 255;
                        line_segment.color.green = 255;
                        line_segment.color.blue = 255;
                        Eigen::Vector3d mp_posi_eig;
                        mp_posi_eig(0)=mp_posis[inliers_mp[i]].x();
                        mp_posi_eig(1)=mp_posis[inliers_mp[i]].y();
                        mp_posi_eig(2)=mp_posis[inliers_mp[i]].z();
                        line_segment.to = mp_posi_eig;
                        //std::cout<<line_segment.to.transpose()<<std::endl;
                        //std::cout<<posi_match_vec.back()<<std::endl;
                        matches.push_back(line_segment);
                    }
                    visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, "global_match_test_line");
                }
                //std::cout<<"match count: "<<inliers_mp.size()<<std::endl;
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    f.close();
    Export_Raw_MatchFile(res_root, time_matchnum_vec);
    return 0;
}