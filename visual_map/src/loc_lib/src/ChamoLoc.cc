#include <loc_lib/ChamoLoc.h>
#ifdef __visualization__
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#endif
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"

#include <ctime>
namespace wayz {
#ifdef __visualization__
    
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
#endif

    void ChamoLoc::StartLocalization(const std::string& filename){
    };
    
    void ChamoLoc::Debug_Image_pose(const double timestamp,const int camera_id, cv::Mat& img_distort)
    {
//         if (!init_state_.isInitialized() || img_distort.empty()) {
//             return;
//         }
//         cv::Mat Img;
//         cv::undistort(img_distort, Img, cam_inter_cv, cam_distort_cv);
//         
//         if(posi_list.size()!=0){
//             double msgTime = timestamp;
//             if (msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_) {
//                 for (int i = 0; i < FilterType::mtState::nCam_; i++) {
//                     if (imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]) {
//                         std::cout
//                             << "    \033[31mFailed Synchronization of Camera Frames, t = "
//                             << msgTime << "\033[0m" << std::endl;
//                     }
//                 }
//                 imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
//             }
//             imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camera_id].computeFromImage(Img, true);
//             imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camera_id] = true;
// 
//             bool measurement_accepted = false;
//             if (imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()) {
//                 measurement_accepted = mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_, msgTime);
//                 imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
//                 //std::thread::id this_id = std::this_thread::get_id();
//             }
//         }
//         std::vector<cv::Point3f> inliers_mp;
//         std::vector<cv::Point2f> inliers_kp;
//         bool loc_re = UpdateByMap(Img, timestamp, inliers_mp, inliers_kp);
//         updateFilter();
//         
//         Eigen::Matrix4d t_mb=Eigen::Matrix4d::Identity();
//         t_mb.block(0,3,3,1)=mpFilter_->safe_.state_.WrWM();
//         t_mb.block(0,0,3,3)=MPD(mpFilter_->safe_.state_.qWM()).matrix();
//         Eigen::Matrix4d t_wm=Eigen::Matrix4d::Identity();
//         t_wm.block(0,3,3,1)=mpFilter_->safe_.state_.poseLin(0);
//         t_wm.block(0,0,3,3)=MPD(mpFilter_->safe_.state_.poseRot(0)).matrix();
//         
//         Eigen::Matrix4d t_re=t_mb;
//         posi_vec.push_back(t_re.block(0,3,3,1));
//         posi_list[timestamp]=t_re.block(0,3,3,1);
//         Eigen::Matrix3d rot_eigen = t_re.block(0,0,3,3);
//         Eigen::Quaterniond rot_q(rot_eigen);
//         rot_list[timestamp]=rot_q;
//         timestamp_list.push_back(timestamp);
//         posi_loc_vec.push_back(t_re.block(0,3,3,1));
//         rot_loc_vec.push_back(rot_q);
//         
//         if(loc_re){
//             cv::Mat debug_img;
//             cv::cvtColor(Img, debug_img, cv::COLOR_GRAY2RGB);
//             for(int i=0; i<inliers_kp.size(); i++){
//                 cv::circle(debug_img, inliers_kp[i], 4, CV_RGB(0,0,255), 2);
//             }
// 
//             img_distort = debug_img.clone();
//             //visualization::RVizVisualizationSink::publish("match_img", debug_img);
//             //visualization::LineSegmentVector matches;
//             for(int i=0; i<inliers_mp.size(); i++){
//                 visualization::LineSegment line_segment;
//                 line_segment.from = posi_match_vec.back();
//                 line_segment.scale = 0.03;
//                 line_segment.alpha = 0.6;
// 
//                 line_segment.color.red = 255;
//                 line_segment.color.green = 255;
//                 line_segment.color.blue = 255;
//                 Eigen::Vector3d mp_posi_eig;
//                 mp_posi_eig(0)=inliers_mp[i].x;
//                 mp_posi_eig(1)=inliers_mp[i].y;
//                 mp_posi_eig(2)=inliers_mp[i].z;
//                 line_segment.to = mp_posi_eig;
// 
//                 fea_match_vec.push_back(mp_posi_eig);
//                 //std::cout<<line_segment.to.transpose()<<std::endl;
//                 //std::cout<<posi_match_vec.back()<<std::endl;
//                 //matches.push_back(line_segment);
//             }
//             //visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, "map_match1");
//         }
        
    }
    void ChamoLoc::Debug_Feature_pose(std::vector<Eigen::Vector3d>& vec)
    {
        if (fea_match_vec.size() > 0)
        {
            vec.swap(fea_match_vec);

            std::vector<Eigen::Vector3d>().swap(fea_match_vec);
        }
    }
    void ChamoLoc::Export_Raw_MatchFile(std::string& path)
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
    void ChamoLoc::AddImage(const double timestamp,const int camera_id, const cv::Mat& img_distort)
    {
        clock_t start = clock();
        cv::Mat Img;
        cv::undistort(img_distort, Img, cam_inter_cv, cam_distort_cv);
        std::vector<int> inliers_mp;
        std::vector<int> inliers_kp;
        ORB_SLAM2::Frame frame;
        chamo::GetAFrame(Img, frame, mpORBextractor, mpVocabulary, cam_inter_cv, cam_distort_cv, "", timestamp);
        Eigen::Matrix4d pose;
        chamo::MatchImg(mp_posis, index_, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
        if(inliers_mp.size()>=20){
            posi_list[timestamp]=pose.block(0,3,3,1);
        }
#ifdef __visualization__
        if(inliers_mp.size()>=20){
            show_mp_as_cloud(posi_match_vec, "loc_lib_posi");
            cv::Mat debug_img;
            cv::cvtColor(Img, debug_img, cv::COLOR_GRAY2RGB);
            for(int i=0; i<inliers_kp.size(); i++){
                cv::circle(debug_img, inliers_kp[i], 4, CV_RGB(0,0,255), 2);
            }
            visualization::RVizVisualizationSink::publish("loc_lib_debug_img", debug_img);
        }
#endif
    };
    
    void ChamoLoc::AddIMU(const double time_s, const Eigen::Vector3d& Accl, const Eigen::Vector3d& Gyro){
        return;
        
    };
    void ChamoLoc::AddMap(const std::string& folder_path){
        vm::VisualMap map;
        vm::loader_visual_map(map, folder_path+"/opti_1000_chamo.map");
        map.GetMPPosiList(mp_posis);
        std::cout<<mp_posis.size()<<std::endl;
#ifdef __visualization__
        show_mp_as_cloud(mp_posis, "temp_mp");
#endif  
        chamo::LoadMap(folder_path, index_, projection_matrix_);
        chamo::GetORBextractor(folder_path, &mpORBextractor, cam_inter_cv, cam_distort_cv);
    };
    
    void ChamoLoc::Shutdown(){
        
    };

    bool ChamoLoc::Debug_QueryPose(const double timestamp, Eigen::Vector3d& Pos, Eigen::Quaterniond& Ori)
    {
        if (posi_match_vec.empty())
        {
            std::cout << "[fail][Debug_QueryPose] posi_loc_queue is empty!" << std::endl;
            return false;
        }
        if (rot_loc_vec.empty())
        {
            std::cout << "[fail][Debug_QueryPose] rot_loc_queue is empty!" << std::endl;
            return false;
        }

        Pos = posi_match_vec.back();
        Ori = rot_loc_vec.back();

        std::vector<Eigen::Vector3d>().swap(posi_match_vec);
        std::vector<Eigen::Quaterniond>().swap(rot_loc_vec);

        return true;
    };

    bool ChamoLoc::QueryPose(const double timestamp, Eigen::Vector3d& Pos, Eigen::Vector3d& Vel, Eigen::Quaterniond& Ori) const{
        if(posi_match_vec.size()>0){
            Pos=(--posi_list.end())->second;
            return true;
        }else{
            return false;
        }
    };
    
}
