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
#include "vis_loc.h"

ORB_SLAM2::ORBVocabulary* mpVocabulary;
ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
ORB_SLAM2::Map* mpMap;
ORB_SLAM2::Tracking* mpTracker;
ORB_SLAM2::LocalMapping* mpLocalMapper;
int img_count;
visual_loc::VisualLocalization VisualLocalization;
std::vector<Eigen::Vector3d> traj;
std::vector<Eigen::Vector3d> local_traj;

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

void showMatchLine(std::vector<Eigen::Vector3d>& posi1, std::vector<Eigen::Vector3d>& posi2, Eigen::Vector3i color, float a, float w, std::string topic){
    visualization::LineSegmentVector matches;
    for(int i=0; i<posi1.size(); i++){
        visualization::LineSegment line_segment;
        line_segment.from = posi2[i];
        line_segment.scale = w;
        line_segment.alpha = a;
        line_segment.color.red = color(0);
        line_segment.color.green = color(1);
        line_segment.color.blue = color(2);
        line_segment.to = posi1[i];
        matches.push_back(line_segment);
    }
    visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, topic);
}

void image_callback(const sensor_msgs::CompressedImageConstPtr& img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    cv::Mat img = cv_ptr->image;
    double timestamp = img_msg->header.stamp.toSec();
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    cv::Mat Tcw;
    std::stringstream img_name;
    img_name<<"img_"<<img_count<<".jpg";
    Eigen::Vector3d output_posi;
    string img_str = img_name.str();
    bool update = VisualLocalization.AddImage(img,img_str,timestamp,output_posi);
    if(update){
        Eigen::Vector3d posi = VisualLocalization.GetGlobalPosition(img_str);
        traj.push_back(posi);
        local_traj.push_back(output_posi);
        show_mp_as_cloud(VisualLocalization.global_posis, "traj");
        // show_mp_as_cloud(local_traj, "local_traj");
        show_mp_as_cloud(VisualLocalization.local_posis, "local_traj");
    }
    
    img_count++;
}

void visMap(){
    std::vector<ORB_SLAM2::MapPoint*> mps_all=mpMap->GetAllMapPoints();
    std::vector<Eigen::Vector3d> mps_eig;
    for(int i=0; i<mps_all.size(); i++){
        cv::Mat center= mps_all[i]->GetWorldPos();
        Eigen::Vector3d center_eig;
        center_eig(0)=center.at<float>(0);
        center_eig(1)=center.at<float>(1);
        center_eig(2)=center.at<float>(2);
        mps_eig.push_back(center_eig);
    }
    show_mp_as_cloud(mps_eig, "temp_mp");

    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    std::vector<Eigen::Vector3d> traj_kf;
    for(int i=0; i<vpKFs.size(); i++){
        cv::Mat center= vpKFs[i]->GetCameraCenter();
        Eigen::Vector3d center_eig;
        center_eig(0)=center.at<float>(0);
        center_eig(1)=center.at<float>(1);
        center_eig(2)=center.at<float>(2);
        traj_kf.push_back(center_eig);
    }
    show_mp_as_cloud(traj_kf, "temp_kf");
    if(0){
        std::vector<Eigen::Vector3d> kf_centers;
        std::vector<Eigen::Vector3d> mp_centers;
        for(int i=0; i<vpKFs.size(); i=i+100){
            cv::Mat center= vpKFs[i]->GetCameraCenter();
            Eigen::Vector3d center_eig;
            center_eig(0)=center.at<float>(0);
            center_eig(1)=center.at<float>(1);
            center_eig(2)=center.at<float>(2);
            std::vector<ORB_SLAM2::MapPoint*> kf_mps= vpKFs[i]->GetMapPointMatches();
            for(int j=0; j<kf_mps.size(); j++){
                cv::Mat center_mp= kf_mps[j]->GetWorldPos();
                Eigen::Vector3d center_mp_eig;
                center_mp_eig(0)=center_mp.at<float>(0);
                center_mp_eig(1)=center_mp.at<float>(1);
                center_mp_eig(2)=center_mp.at<float>(2);
                mp_centers.push_back(center_mp_eig);
                kf_centers.push_back(center_eig);
            }
        }
        showMatchLine(mp_centers, kf_centers, Eigen::Vector3i(255,255,255),0.6, 0.03, "temp_match_mp");
        
        std::vector<Eigen::Vector3d> kf_centers1;
        std::vector<Eigen::Vector3d> kf_centers2;
        for(int i=0; i<vpKFs.size(); i=i+10){
            std::vector<ORB_SLAM2::KeyFrame*> kfs= vpKFs[i]->GetVectorCovisibleKeyFrames();
            cv::Mat center= vpKFs[i]->GetCameraCenter();
            Eigen::Vector3d center_eig;
            center_eig(0)=center.at<float>(0);
            center_eig(1)=center.at<float>(1);
            center_eig(2)=center.at<float>(2);
            for(int j=0; j<kfs.size(); j++){
                cv::Mat center2 = kfs[j]->GetCameraCenter();
                Eigen::Vector3d center2_eig;
                center2_eig(0)=center2.at<float>(0);
                center2_eig(1)=center2.at<float>(1);
                center2_eig(2)=center2.at<float>(2);
                kf_centers1.push_back(center_eig);
                kf_centers2.push_back(center2_eig);
            }
        }
        showMatchLine(kf_centers1, kf_centers2, Eigen::Vector3i(255,255,0),1, 0.03, "temp_match_kf");
        
        kf_centers1.clear();
        kf_centers2.clear();
        for(int i=0; i<vpKFs.size(); i=i+1){
            ORB_SLAM2::KeyFrame* kf2= vpKFs[i]->GetParent();
            if(kf2!= NULL){
                cv::Mat center= vpKFs[i]->GetCameraCenter();
                Eigen::Vector3d center_eig;
                center_eig(0)=center.at<float>(0);
                center_eig(1)=center.at<float>(1);
                center_eig(2)=center.at<float>(2);
                cv::Mat center2 = kf2->GetCameraCenter();
                Eigen::Vector3d center2_eig;
                center2_eig(0)=center2.at<float>(0);
                center2_eig(1)=center2.at<float>(1);
                center2_eig(2)=center2.at<float>(2);
                kf_centers1.push_back(center_eig);
                kf_centers2.push_back(center2_eig);
            }
        }
        showMatchLine(kf_centers1, kf_centers2, Eigen::Vector3i(255,0,255),1, 0.3, "temp_span_kf");
    }
}

                                        
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string work_root = argv[1];
    std::string bag_addr  = argv[2];
    VisualLocalization.SetInit(work_root);
    VisualLocalization.LoadMapLabMap();

    img_count=0;

    
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("img");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count= 0;
    int skip = 0;
    rosbag::View::iterator it= view.begin();

    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        // if(skip++ < 3390) continue;
        // if(skip++ % 3 != 0) continue;
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                cv::Mat img= cv_ptr->image;
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                double timestamp = simg->header.stamp.toSec();
                std::stringstream img_name;
                img_name<<"img_"<<img_count<<".jpg";
                LOG(INFO)<<img_name.str();
                Eigen::Vector3d output_posi;
                string img_str = img_name.str();
                bool update = VisualLocalization.AddImage(img,img_str,timestamp,output_posi);
                if(update){
                    Eigen::Vector3d posi = VisualLocalization.GetGlobalPosition(img_str);
                    traj.push_back(posi);
                    local_traj.push_back(output_posi);
                    // show_mp_as_cloud(VisualLocalization.global_posis, "global_traj");
                    show_mp_as_cloud(VisualLocalization.local_posis, "local_traj");
                    show_mp_as_cloud(traj,"global_traj");
                    std::vector<Eigen::Quaterniond> global_rot,local_rot;
                    std::vector<Eigen::Vector3d> global_tran,local_tran;
                    for(int i = 0; i < VisualLocalization.global_poses.size(); i++)
                    {
                        Eigen::Matrix3d rot = VisualLocalization.global_poses[i].block(0,0,3,3);
                        Eigen::Quaterniond q(rot);
                        Eigen::Vector3d tran = VisualLocalization.global_poses[i].block(0,3,3,1);
                        global_rot.push_back(q);
                        global_tran.push_back(tran);
                    }
                    for(int i = 0; i < VisualLocalization.local_poses.size(); i++)
                    {
                        Eigen::Matrix3d rot = VisualLocalization.local_poses[i].block(0,0,3,3);
                        Eigen::Quaterniond q(rot);
                        Eigen::Vector3d tran = VisualLocalization.local_poses[i].block(0,3,3,1);
                        local_rot.push_back(q);
                        local_tran.push_back(tran);
                    }
                    show_pose_as_marker(global_tran, global_rot, "global");
                    show_pose_as_marker(local_tran, local_rot, "local");
                    show_mp_as_cloud(local_traj,"realtime");
                    show_mp_as_cloud(VisualLocalization.mp_before, "mp_before");
                    show_mp_as_cloud(VisualLocalization.mp_after, "mp_after");
                }
                
                img_count++;
                
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }

    return 0;
}
