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
#include <glog/logging.h>
#include <gflags/gflags.h>
#include"Frame.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include "global_match/orb_match.h"
#include "global_match/global_match.h"
#include "orb_slam_lib/sim3_match.h"
#include "CoorConv.h"
#include "vis_loc.h"

DEFINE_string(map1_addr, "", "First map to merge.");
DEFINE_string(map2_addr, "", "Second map to merge.");
DEFINE_string(desc1_addr, "", "Descriptor index of first map to merge.");
DEFINE_string(desc2_addr, "", "Descriptor index of second map to merge.");
DEFINE_string(output_addr, "", "Address for the output map.");

              
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

void transformPoseUseSim3(Eigen::Matrix4d& sim3, double scale,  Eigen::Matrix4d& in_pose,  Eigen::Matrix4d& out_pose){
    Eigen::Matrix3d R_tran=sim3.block(0,0,3,3)/scale;
    Eigen::Matrix3d R_in=in_pose.block(0,0,3,3);
    Eigen::Matrix3d R_out=R_tran*R_in;
    Eigen::Vector4d t_out=sim3*in_pose.block(0,3,4,1);
    out_pose= Eigen::Matrix4d::Identity();
    out_pose.block(0,0,3,3) = R_out;
    out_pose.block(0,3,4,1) = t_out;
}

void convertToFrame(std::shared_ptr<vm::Frame> frame_vm, ORB_SLAM2::Frame& frame){
    int desc_width=frame_vm->descriptors.rows();
    int desc_count=frame_vm->descriptors.cols();
    frame.mDescriptors=cv::Mat(desc_count, desc_width , CV_8UC1);
    frame.mvKeysUn=frame_vm->kps;
    for(int i=0; i<desc_count; i++){
        for(int j=0; j<desc_width; j++){
            frame.mDescriptors.at<unsigned char>(i, j)=frame_vm->descriptors(j, i);
            //std::cout<<(int)frame.mDescriptors.at<unsigned char>(i, j)<<",";
        }
        //std::cout<<std::endl;
    }
    frame.mDistCoef=cv::Mat::zeros(1,4,CV_32FC1);
    frame.mK=cv::Mat::eye(3,3,CV_32FC1);
    frame.mK.at<float>(0,0)=frame_vm->fx;
    frame.mK.at<float>(1,1)=frame_vm->fy;
    frame.mK.at<float>(0,2)=frame_vm->cx;
    frame.mK.at<float>(1,2)=frame_vm->cy;
}

struct KP_INFO{
    std::shared_ptr<vm::Frame> frame;
    int kp_ind;
    std::vector<std::shared_ptr<vm::MapPoint>> mps;
};

void doAMatch(std::vector<KP_INFO>& kp_info_list, vm::VisualMap& source_map, vm::VisualMap& target_map, std::string target_desc_file,
    std::vector<Eigen::Matrix4d>& T_tar_sour_list, std::vector<double>& scale_tar_sour_list, 
    std::vector<std::shared_ptr<vm::Frame>>& graph_tar_list, std::vector<std::shared_ptr<vm::Frame>>& graph_sour_list,
    Eigen::Matrix4d& sim3_tar_sour, double& scale_tar_sour
){
    visual_loc::VisualLocalization visloc_t;
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index;
    Eigen::MatrixXf projection_matrix_;
    chamo::LoadMap(target_desc_file, index, projection_matrix_);
    std::vector<Eigen::Vector3d> mp_posis;
    target_map.GetMPPosiList(mp_posis);
    std::vector<shared_ptr<vm::MapPoint>> pc_match_sour;
    std::vector<shared_ptr<vm::MapPoint>> pc_match_tar;
    for(int i=0; i<source_map.frames.size(); i++){
        ORB_SLAM2::Frame frame;
        convertToFrame(source_map.frames[i], frame);
        std::vector<int> inliers_mp;
        std::vector<int> inliers_kp;
        Eigen::Matrix4d pose;
        chamo::MatchImg(mp_posis, index, projection_matrix_, frame, inliers_mp, inliers_kp, pose);
        std::vector<Eigen::Vector3d> local_pc;
        std::vector<Eigen::Vector3d> global_pc;
        std::vector<shared_ptr<vm::MapPoint>> pc_match_sour_temp;
        std::vector<shared_ptr<vm::MapPoint>> pc_match_tar_temp;
        for(int j=0; j<inliers_kp.size(); j++){
            if(source_map.frames[i]->obss[inliers_kp[j]]!=nullptr){
                local_pc.push_back(source_map.frames[i]->obss[inliers_kp[j]]->position);
                global_pc.push_back(mp_posis[inliers_mp[j]]);
                pc_match_sour_temp.push_back(source_map.frames[i]->obss[inliers_kp[j]]);
                pc_match_tar_temp.push_back(target_map.mappoints[inliers_mp[j]]);
            }
        }
        
        if(local_pc.size()>30){
            for(int j=0; j<pc_match_sour_temp.size(); j++){
                bool find_one=false;
                for(int k=0; k<pc_match_sour.size();k++){
                    if(pc_match_sour_temp[j]==pc_match_sour[k]){
                        if(pc_match_tar_temp[j]==pc_match_tar[k]){
                            find_one=true;
                            break;
                        }
                    }
                }
                if(find_one==false){
                    pc_match_sour.push_back(pc_match_sour_temp[j]);
                    pc_match_tar.push_back(pc_match_tar_temp[j]);
                }
            }
            
            //LOG(INFO)<<"local_pc.size(): "<<local_pc.size();
            Eigen::Matrix4d T_tar_sour;
            double scale_tar_sour; 
            bool succ= visloc_t.ComputeSim3Ransac(global_pc, local_pc, T_tar_sour, scale_tar_sour);
            if(succ){
                //orb_slam::ComputeSim3(global_pc, local_pc , T_tar_sour, scale_tar_sour);
                //LOG(INFO)<<"scale_tar_sour: "<<scale_tar_sour;
                std::map<std::shared_ptr<vm::Frame>, int> frame_list;
                for(int j=0; j<inliers_mp.size(); j++){
                    std::shared_ptr<vm::MapPoint> temp_tar_mp = target_map.mappoints[inliers_mp[j]];
                    for(int k=0; k<temp_tar_mp->track.size(); k++){
                        if(frame_list.count(temp_tar_mp->track[k].frame)==0){
                            frame_list[temp_tar_mp->track[k].frame]=1;
                        }else{
                            frame_list[temp_tar_mp->track[k].frame]=frame_list[temp_tar_mp->track[k].frame]+1;
                        }
                    }
                }
                
                std::vector<std::shared_ptr<vm::Frame> > connected_frames;
                std::map<std::shared_ptr<vm::Frame>, int>::iterator it;
                int max_count=0;
                std::shared_ptr<vm::Frame> max_frame=nullptr;
                for ( it = frame_list.begin(); it != frame_list.end(); it++ ){
//                     if(it->second>60){
//                         connected_frames.push_back(it->first);
//                     }
                    if(it->second>max_count){
                        max_count=it->second;
                        max_frame=it->first;
                    }
                }
                if(max_frame!= nullptr && max_count>60){
                    connected_frames.push_back(max_frame);
                }
                //LOG(INFO)<<"connected_frames: "<<connected_frames.size();
                for(int j=0; j<connected_frames.size(); j++){
                    graph_tar_list.push_back(connected_frames[j]);
                    graph_sour_list.push_back(source_map.frames[i]);
                    scale_tar_sour_list.push_back(scale_tar_sour);
                    Eigen::Matrix4d T_tarworld_sourworld = T_tar_sour;
                    Eigen::Matrix4d T_tarworld_tar = connected_frames[j]->getPose();
                    Eigen::Matrix4d T_sourworld_sour = source_map.frames[i]->getPose();
                    Eigen::Matrix4d T_tar_tarworld = T_tarworld_tar.inverse();
                    T_tar_sour_list.push_back(T_tar_tarworld*T_tarworld_sourworld*T_sourworld_sour);
                }
//                 Eigen::Matrix4d T_sourworld_sour = source_map.frames[i]->getPose();
//                 Eigen::Vector4d posi_homo;
//                 posi_homo.block(0,0,3,1)=T_sourworld_sour.block(0,3,3,1);
//                 posi_homo(3)=1;
//                 Eigen::Vector4d sim_posi = T_tar_sour*posi_homo;
                //LOG(INFO)<<"sim_posi: "<<sim_posi.block(0,0,3,1).transpose();
            }
            
        }
        
        for(int j=0; j<inliers_kp.size(); j++){
            CHECK_GT(source_map.frames[i]->obss.size(), inliers_kp[j]);
            std::shared_ptr<vm::MapPoint> mp1=source_map.frames[i]->obss[inliers_kp[j]];
            if(mp1!=nullptr){
                CHECK_GT(target_map.mappoints.size(), inliers_mp[j]);
                std::shared_ptr<vm::MapPoint> mp2=target_map.mappoints[inliers_mp[j]];
                CHECK_NOTNULL(mp2);
                int find_one_id=-1;
                for(int k=0; k<kp_info_list.size(); k++){
                    if(kp_info_list[k].frame==source_map.frames[i] && kp_info_list[k].kp_ind == inliers_kp[j]){
                        find_one_id=k;
                        break;
                    }
                }
                if(find_one_id==-1){
                    KP_INFO kp_info;
                    kp_info.frame=source_map.frames[i];
                    kp_info.kp_ind=inliers_kp[j];
                    kp_info.mps.push_back(mp1);
                    kp_info.mps.push_back(mp2);
                    kp_info_list.push_back(kp_info);
                }else{
                    //LOG(INFO)<<"use existed kp_info: "<<kp_info_list[find_one_id].frame->id;
                    kp_info_list[find_one_id].mps.push_back(mp2);
                }
            }
        }
    }
    
    std::vector<Eigen::Vector3d> posi_sour_temp;
    std::vector<Eigen::Vector3d> posi_tar_temp;
    
    for(int i=0; i< pc_match_sour.size(); i++){
        posi_sour_temp.push_back(pc_match_sour[i]->position);
        posi_tar_temp.push_back(pc_match_tar[i]->position);
    }
    bool succ= visloc_t.ComputeSim3Ransac(posi_tar_temp, posi_sour_temp, sim3_tar_sour, scale_tar_sour);
}

void simpleMerge(vm::VisualMap& base_map, vm::VisualMap& to_merge_map){
    for(int i=0; i<to_merge_map.frames.size(); i++){
        base_map.frames.push_back(to_merge_map.frames[i]);
    }
    for(int i=0; i<to_merge_map.mappoints.size(); i++){
        base_map.mappoints.push_back(to_merge_map.mappoints[i]);
    }
    base_map.pose_graph_v1.insert(base_map.pose_graph_v1.begin(),to_merge_map.pose_graph_v1.begin(),to_merge_map.pose_graph_v1.end());
    base_map.pose_graph_v2.insert(base_map.pose_graph_v2.begin(),to_merge_map.pose_graph_v2.begin(),to_merge_map.pose_graph_v2.end());
    base_map.pose_graph_e_posi.insert(base_map.pose_graph_e_posi.begin(),to_merge_map.pose_graph_e_posi.begin(),to_merge_map.pose_graph_e_posi.end());
    base_map.pose_graph_e_rot.insert(base_map.pose_graph_e_rot.begin(),to_merge_map.pose_graph_e_rot.begin(),to_merge_map.pose_graph_e_rot.end());
    base_map.pose_graph_e_scale.insert(base_map.pose_graph_e_scale.begin(),to_merge_map.pose_graph_e_scale.begin(),to_merge_map.pose_graph_e_scale.end());
    base_map.pose_graph_weight.insert(base_map.pose_graph_weight.begin(),to_merge_map.pose_graph_weight.begin(),to_merge_map.pose_graph_weight.end());
}

int findInKPList(std::vector<KP_INFO>& kp_info_list, vm::TrackItem track){
    for(int i=0; i<kp_info_list.size(); i++){
        if(kp_info_list[i].frame==track.frame && kp_info_list[i].kp_ind == track.kp_ind){
            return i;
        }
    }
    return -1;
}

void MergeMP(std::shared_ptr<vm::MapPoint> base_mp, std::shared_ptr<vm::MapPoint> to_merge_mp){
    for(int k=0; k<to_merge_mp->track.size(); k++){
        CHECK_GT(to_merge_mp->track[k].frame->obss.size(), to_merge_mp->track[k].kp_ind);
        to_merge_mp->track[k].frame->obss[to_merge_mp->track[k].kp_ind]=base_mp;
    }
    
}

            
int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    
    vm::VisualMap map1;
    vm::loader_visual_map(map1, FLAGS_map1_addr);
    map1.ComputeUniqueId();
    vm::VisualMap map2;
    vm::loader_visual_map(map2, FLAGS_map2_addr);
    map2.ComputeUniqueId();
    std::vector<Eigen::Matrix4d> T_tar_sour_list;
    std::vector<double> scale_tar_sour_list;
    std::vector<KP_INFO> kp_info_list;
    std::vector<std::shared_ptr<vm::Frame>> graph_tar_list;
    std::vector<std::shared_ptr<vm::Frame>> graph_sour_list;
    Eigen::Matrix4d sim3_2_1;
    double scale_2_1;
    doAMatch(kp_info_list, map1, map2, FLAGS_desc2_addr, T_tar_sour_list, scale_tar_sour_list, graph_tar_list, graph_sour_list, sim3_2_1, scale_2_1);
    LOG(INFO)<<"kp_info_list: "<<kp_info_list.size();
    LOG(INFO)<<"scale_2_1: "<<scale_2_1;
    Eigen::Matrix4d sim3_1_2;
    double scale_1_2;
    doAMatch(kp_info_list, map2, map1, FLAGS_desc1_addr, T_tar_sour_list, scale_tar_sour_list, graph_tar_list, graph_sour_list, sim3_1_2, scale_1_2);
    LOG(INFO)<<"kp_info_list: "<<kp_info_list.size();
    LOG(INFO)<<"scale_1_2: "<<scale_1_2;
    
//     for(int i=0; i<map2.frames.size(); i++){
//         Eigen::Matrix4d pose_transformed_temp;
//         Eigen::Matrix4d temp_pose=map2.frames[i]->getPose();
//         transformPoseUseSim3(sim3_1_2, scale_1_2, temp_pose, pose_transformed_temp);
//         map2.frames[i]->setPose(pose_transformed_temp);
//     }
//     for(int j=0; j<map2.mappoints.size(); j++){
//         Eigen::Vector4d posi_homo;
//         posi_homo.block(0,0,3,1)=map2.mappoints[j]->position;
//         posi_homo(3)=1;
//         Eigen::Vector4d posi_gps_homo = sim3_1_2*posi_homo;
//         map2.mappoints[j]->position=posi_gps_homo.block(0,0,3,1);                       
//     }
    
    GpsConverter gps_conv1(map1.gps_anchor(0), map1.gps_anchor(1), false);
    GpsConverter gps_conv2(map2.gps_anchor(0), map2.gps_anchor(1), false);
    
    for(int i=0; i<map2.frames.size(); i++){
        WGS84Corr latlon;
        gps_conv2.MapXYToLatLon(map2.frames[i]->position.x(), map2.frames[i]->position.y(), latlon);
        UTMCoor xy;
        gps_conv1.MapLatLonToXY(latlon.lat, latlon.log, xy);
        map2.frames[i]->position.x()=xy.x;
        map2.frames[i]->position.y()=xy.y;
    }
    
    for(int i=0; i<map2.frames.size(); i++){
        WGS84Corr latlon;
        gps_conv2.MapXYToLatLon(map2.frames[i]->gps_position.x(), map2.frames[i]->gps_position.y(), latlon);
        UTMCoor xy;
        gps_conv1.MapLatLonToXY(latlon.lat, latlon.log, xy);
        map2.frames[i]->gps_position.x()=xy.x;
        map2.frames[i]->gps_position.y()=xy.y;
    }
    
    for(int i=0; i<map2.mappoints.size(); i++){
        WGS84Corr latlon;
        gps_conv2.MapXYToLatLon(map2.mappoints[i]->position.x(), map2.mappoints[i]->position.y(), latlon);
        UTMCoor xy;
        gps_conv1.MapLatLonToXY(latlon.lat, latlon.log, xy);
        map2.mappoints[i]->position.x()=xy.x;
        map2.mappoints[i]->position.y()=xy.y;
    }
    
//     for(int i=0; i<map1.frames.size()-1; i++){
//         T_tar_sour_list.push_back(map1.frames[i+1]->getPose().inverse()*map1.frames[i]->getPose());
//         scale_tar_sour_list.push_back(1);
//         //std::cout<<(map1.frames[i+1]->getPose().block(0,0,3,3)*T_tar_sour_list.back().block(0,3,3,1)+map1.frames[i+1]->getPose().block(0,3,3,1)).transpose()<<std::endl;
//         //std::cout<<map1.frames[i]->getPose().block(0,3,3,1).transpose()<<std::endl;
//         //std::cout<<"sdfasd"<<std::endl;
//         graph_sour_list.push_back(map1.frames[i]);
//         graph_tar_list.push_back(map1.frames[i+1]);
//     }
//     
//     for(int i=0; i<map2.frames.size()-1; i++){
//         T_tar_sour_list.push_back(map2.frames[i+1]->getPose().inverse()*map2.frames[i]->getPose());
//         scale_tar_sour_list.push_back(1);
//         graph_sour_list.push_back(map2.frames[i]);
//         graph_tar_list.push_back(map2.frames[i+1]);
//         
//     }
    simpleMerge(map1, map2);
    for(int i=0; i<T_tar_sour_list.size(); i++){
        Eigen::Matrix3d rot=T_tar_sour_list[i].block(0,0,3,3)/scale_tar_sour_list[i];
        Eigen::Vector3d posi=T_tar_sour_list[i].block(0,3,3,1);
        map1.AddConnection(graph_sour_list[i], graph_tar_list[i], posi,rot, scale_tar_sour_list[i], 100);
    }
    map1.ComputeUniqueId();
    map1.CheckConsistence();
    std::vector<int> merged_mps;
    
    for(int i=0; i<map1.mappoints.size(); i++){
        merged_mps.push_back(-1);
    }
    int merged_mp_count=0;
    for(int i=0; i<map1.mappoints.size(); i++){
        if(merged_mps[i]!=-1){
            continue;
        }
        int track_size=map1.mappoints[i]->track.size();
        for(int j=0; j<track_size; j++){
            int temp_ip_id=findInKPList(kp_info_list, map1.mappoints[i]->track[j]);
            //LOG(INFO)<<"temp_ip_id:"<<temp_ip_id;
            //LOG(INFO)<<"map1.mappoints[i]->track[j]:"<<map1.mappoints[i]->track[j].kp_ind;
            if(temp_ip_id!=-1){ 
                for(int k=0; k<kp_info_list[temp_ip_id].mps.size(); k++){
                    if(merged_mps[kp_info_list[temp_ip_id].mps[k]->id]!=-1){
                        continue;
                    }
                    if(map1.mappoints[i]->id==kp_info_list[temp_ip_id].mps[k]->id){
                        continue;
                    }
                    MergeMP(map1.mappoints[i], kp_info_list[temp_ip_id].mps[k]);
                    merged_mps[kp_info_list[temp_ip_id].mps[k]->id]=0;
                    merged_mp_count++;
                    merged_mps[i]=1;
                }
            }
        }
    }
    LOG(INFO)<<"merged mp count: "<<merged_mp_count;
    for(int i=map1.mappoints.size()-1; i>=0; i--){
        if(merged_mps[i]==0){
            map1.mappoints.erase(map1.mappoints.begin()+i);
        }
    }
    map1.ComputeUniqueId();
    map1.AssignKpToMp();
    map1.CheckConsistence();
    vm::save_visual_map(map1, FLAGS_output_addr);

    return 0;
}