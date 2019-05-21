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

double pose_time;
bool init_flag = false;

ORB_SLAM2::ORBVocabulary* mpVocabulary;
ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;
ORB_SLAM2::Map* mpMap;
ORB_SLAM2::Tracking* mpTracker;
ORB_SLAM2::LocalMapping* mpLocalMapper;
int img_count;

std::vector<Eigen::Vector3d> re_traj;

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
    std::stringstream ss;
    ss<<"img_"<<img_count<<".jpg";
    cv::Mat Tcw = mpTracker->Loc(img,timestamp, ss.str());
    if(!Tcw.empty()){
        cv::Mat Twc= Tcw.inv();
        Eigen::Vector3d re_posi;
        re_posi(0)=Twc.at<float>(0,3);
        re_posi(1)=Twc.at<float>(1,3);
        re_posi(2)=Twc.at<float>(2,3);
        re_traj.push_back(re_posi);
        show_mp_as_cloud(re_traj, "temp_re");
    }
    
    img_count++;
}

void findIdByName(std::vector<std::string>& names, int& re_id, std::string query_name){
    re_id=-1;
    for(int i=0; i<names.size(); i++){
        if(names[i]==query_name){
            re_id=i;
            return;
        }
    }
    return;
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
    std::string res_root=argv[1];
    std::string strVocFile=res_root+"/orbVoc.bin";
    mpVocabulary = new ORB_SLAM2::ORBVocabulary();
    bool bVocLoad= mpVocabulary->loadFromBinaryFile(strVocFile);
    if(bVocLoad==false){
        std::cout<<"try binary voc failed, use txt format to load."<<std::endl;
        mpVocabulary->load(strVocFile);
    }
    
    mpKeyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*mpVocabulary);
    mpMap = new ORB_SLAM2::Map();
    mpTracker = new ORB_SLAM2::Tracking(mpVocabulary, mpMap, mpKeyFrameDatabase, res_root+"/vslam.yaml",0 ,false);
    mpLocalMapper = new ORB_SLAM2::LocalMapping(mpMap, true);

    mpTracker->SetLocalMapper(mpLocalMapper);
    mpLocalMapper->SetTracker(mpTracker);
    
    std::string cam_addr=res_root+"/camera_config.txt";
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    CHAMO::read_cam_info(cam_addr, cam_inter, cam_distort, Tbc);
    std::cout<<"cam_inter: "<<cam_inter<<std::endl;
    
    std::string image_config_addr=res_root+"/image_conf.txt";
    int width;
    int height; 
    float desc_scale;
    int desc_level; 
    int desc_count;
    CHAMO::read_image_info(image_config_addr, width, height, desc_scale, desc_level, desc_count);
    std::cout<<"image_conf: "<<width<<":"<<height<<std::endl;
    std::vector<Eigen::Matrix4d> poses_alin;
    std::vector<std::string> frame_names;
    std::string traj_file_addr = res_root+"/frame_pose_opt.txt";
    CHAMO::read_traj_file(traj_file_addr, poses_alin, frame_names);
    std::cout<<"frame_pose_opt: "<<poses_alin.size()<<std::endl;
    
    std::string img_time_addr=res_root+"/image_time.txt";
    std::vector<double> img_timess;
    std::vector<std::string> imgtime_names;
    CHAMO::read_img_time(img_time_addr, img_timess, imgtime_names);
    std::cout<<"img_timess: "<<img_timess.size()<<std::endl;
    
    std::string gps_alin_addr=res_root+"/gps_alin.txt";
    std::vector<int> gps_inliers;
    std::vector<Eigen::Vector3d> gps_alins;
    CHAMO::read_gps_alin(gps_alin_addr, gps_alins, gps_inliers);
    
    std::string kp_addr=res_root+"/kps.txt";
    std::vector<Eigen::Vector2f> kp_uvs;
    std::vector<std::string> kp_framename;
    std::vector<int> kp_octoves;
    CHAMO::read_kp_info(kp_addr, kp_uvs, kp_framename, kp_octoves);
    std::cout<<"kp_uvs: "<<kp_uvs.size()<<std::endl;
    
    std::string posi_addr=res_root+"/mp_posi_opt.txt";
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    std::cout<<"mp_posis: "<<mp_posis.size()<<std::endl;
    
    std::string track_addr=res_root+"/track.txt";
    std::vector<std::vector<int>> tracks;
    CHAMO::read_track_info(track_addr, tracks);
    std::cout<<"tracks: "<<tracks.size()<<std::endl;
    
    std::string desc_addr=res_root+"/desc.txt";
    std::vector<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>> descs;
    CHAMO::read_desc_eigen(desc_addr, descs);
    
    std::vector<float> cam_info;
    cam_info.push_back(cam_inter(0,0));
    cam_info.push_back(cam_inter(1,1));
    cam_info.push_back(cam_inter(2,0));
    cam_info.push_back(cam_inter(2,1));
    cam_info.push_back(width);
    cam_info.push_back(height);
    std::vector<ORB_SLAM2::KeyFrame*> kfs;
    for(int i=0; i<poses_alin.size(); i++){
        int time_id;
        findIdByName(imgtime_names, time_id, frame_names[i]);
        if(time_id==-1){
            std::cout<<"imgtime_names find error: "<<frame_names[i]<<std::endl;
            return 0;
        }
        ORB_SLAM2::KeyFrame* pKF = new ORB_SLAM2::KeyFrame();
        std::vector<cv::KeyPoint> keysUn;
        cv::Mat descriptors;
        cv::Mat pose_c_w=ORB_SLAM2::Converter::toCvMat(poses_alin[i]).inv();
        pKF->setData(i, img_timess[time_id], keysUn, cam_info, frame_names[i], desc_level, desc_scale, pose_c_w, 
                     descriptors, mpMap, mpKeyFrameDatabase, mpVocabulary);
        mpMap->AddKeyFrame(pKF);
        kfs.push_back(pKF);
    }
    
    std::map<std::string, int> frame_names_to_ids;
    
    for(int i=0; i<frame_names.size(); i++){
        frame_names_to_ids[frame_names[i]]=i;
    }

    for(int i=0; i<tracks.size(); i++){
        int mp_id=i;
        ORB_SLAM2::MapPoint* pMP=NULL;
        for (int j=0; j<tracks[i].size(); j++){
            //std::cout<<j<<":"<<tracks[i].size()<<std::endl;
            int kp_id=tracks[i][j];
            int kpframe_id;
            findIdByName(frame_names, kpframe_id, kp_framename[kp_id]);
            //kpframe_id=frame_names_to_ids[kp_framename[kp_id]];
            if(kpframe_id==-1){
                continue;
            }
            
            if(pMP==NULL){
                pMP = new ORB_SLAM2::MapPoint(ORB_SLAM2::Converter::toCvMat(mp_posis[mp_id]),kfs[kpframe_id],mpMap);
            }
            
            cv::KeyPoint kp;
            kp.pt.x=kp_uvs[kp_id](0);
            kp.pt.y=kp_uvs[kp_id](1);
            kp.octave=kp_octoves[kp_id];
            
            cv::Mat desc(1,descs[kp_id].rows(), CV_8UC1);
            for(int k=0; k<descs[kp_id].rows(); k++){
                desc.at<unsigned char>(k)=descs[kp_id](k, 0);
            }
            int kp_inframe_id = kfs[kpframe_id]->AddKP(kp, desc);
            kfs[kpframe_id]->AddMapPoint(pMP,kp_inframe_id);
            pMP->AddObservation(kfs[kpframe_id],kp_inframe_id);     
        }
        mpMap->AddMapPoint(pMP);
    } 
    
    std::vector<ORB_SLAM2::MapPoint*> mps_all=mpMap->GetAllMapPoints();
    for(int i=0; i<mps_all.size(); i++){
        mps_all[i]->ComputeDistinctiveDescriptors();
        mps_all[i]->UpdateNormalAndDepth();
    }
    
    vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it){
        (*it)->finishDataSetting();
        mpKeyFrameDatabase->add((*it));
    }
    for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it){
        (*it)->UpdateConnections();
    }
    std::cout<<"map loaded!"<<std::endl;
    visMap();
    
    
    img_count=0;
        
    ros::Rate loop_rate(1000);
    ros::Subscriber img_subscriber_;
    ros::Publisher  pose_pub;
    img_subscriber_ = nh.subscribe("img", 1, image_callback);
    
    ros::start();
    Eigen::Vector3d Pos;
    Eigen::Vector3d Vel;
    Eigen::Quaterniond Ori;
    int counter = 0;
    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    

    return 0;
}