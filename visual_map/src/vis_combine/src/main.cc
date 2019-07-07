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
#include "optimizer_tool/optimizer_tool.h"

DEFINE_string(map_root_addr, "", "First map to merge.");

              
void printFrameInfo(ORB_SLAM2::Frame& frame){
    std::cout<<"kp count: "<<frame.mvKeysUn.size()<<std::endl;
    std::cout<<"desc size (w:h) "<<frame.mDescriptors.cols<<":"<<frame.mDescriptors.rows<<std::endl;
}

void transformPoseUseSim3(Eigen::Matrix4d& sim3, Eigen::Matrix4d& in_pose,  Eigen::Matrix4d& out_pose){
    
    Eigen::Matrix3d R_tran=sim3.block(0,0,3,3);
    double scale=R_tran.block(0,0,3,1).norm();
    R_tran=R_tran/scale;
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

bool doAMatch(std::vector<KP_INFO>& kp_info_list, vm::VisualMap& source_map, vm::VisualMap& target_map, std::string index_filename, std::string index_dir,
    std::vector<Eigen::Matrix4d>& T_tar_sour_list, std::vector<double>& scale_tar_sour_list, 
    std::vector<std::shared_ptr<vm::Frame>>& graph_tar_list, std::vector<std::shared_ptr<vm::Frame>>& graph_sour_list,
    Eigen::Matrix4d& sim3_tar_sour, double& scale_tar_sour
){
    visual_loc::VisualLocalization visloc_t;
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index;
    Eigen::MatrixXf projection_matrix_;
    chamo::LoadMap(index_dir, index, projection_matrix_, index_filename);
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
            std::vector<cv::KeyPoint> local_kp;
            cv::Mat mk;
            bool succ= visloc_t.ComputeSim3Ransac(global_pc, local_pc, local_kp, mk, T_tar_sour, scale_tar_sour);
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
            }else{
                //std::cout<<"ransac for local pc match failed!!"<<std::endl;
                //return false;
            }
            
        }else{
            //std::cout<<"not enouph local_pc!!"<<std::endl;
            //return false;
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
    if(pc_match_sour.size()<50){
        std::cout<<"too few pc: "<<pc_match_sour.size()<<std::endl;
        return false;
    }
    for(int i=0; i< pc_match_sour.size(); i++){
        posi_sour_temp.push_back(pc_match_sour[i]->position);
        posi_tar_temp.push_back(pc_match_tar[i]->position);
    }
    std::vector<cv::KeyPoint> local_kp;
    cv::Mat mk;
    bool succ= visloc_t.ComputeSim3Ransac(posi_tar_temp, posi_sour_temp,local_kp,mk, sim3_tar_sour, scale_tar_sour);
    if(succ){
        
    }else{
        std::cout<<"ransac for pc match failed!!"<<std::endl;
    }
    return succ;
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

void ExpandANode(int map_id, std::vector<int>& expended_list, std::vector<int>& v1_list, std::vector<int>& v2_list, 
                 std::vector<int>& p_list, std::vector<int>& c_list, 
                 std::vector<Eigen::Matrix4d>& T_c_ps, std::vector<Eigen::Matrix4d>& T_2_1s){
    expended_list[map_id]=1;
    for(int i=0; i<v1_list.size(); i++){
        if(map_id==v1_list[i]){
            if(expended_list[v2_list[i]]!=1){
                p_list.push_back(map_id);
                c_list.push_back(v2_list[i]);
                T_c_ps.push_back(T_2_1s[i]);
                ExpandANode(v2_list[i], expended_list, v1_list, v2_list, p_list, c_list, T_c_ps, T_2_1s);
            }
        }
        if(map_id==v2_list[i]){
            if(expended_list[v1_list[i]]!=1){
                p_list.push_back(map_id);
                c_list.push_back(v1_list[i]);
                T_c_ps.push_back(T_2_1s[i].inverse());
                ExpandANode(v1_list[i], expended_list, v1_list, v2_list, p_list, c_list, T_c_ps, T_2_1s);
            }
        }
    }
}

            
int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    
    std::vector<std::shared_ptr<vm::VisualMap>> maps;
    std::vector<std::string> desc_index_files;
    for(int i=0; i<100; i++){
        std::stringstream ss;
        ss<<FLAGS_map_root_addr<<"/chamo_"<<i<<".map";
        std::shared_ptr<vm::VisualMap> map;
        map.reset(new vm::VisualMap);
        bool re =vm::loader_visual_map(*map, ss.str());
        if(re){
            map->ComputeUniqueId();
            maps.push_back(map);
            std::stringstream ss1;
            ss1<<"chamo_"<<i<<".map.desc";
            desc_index_files.push_back(ss1.str());
        }
    }
    
    std::vector<KP_INFO> kp_info_list;
    std::vector<Eigen::Matrix4d> T_tar_sour_list;
    std::vector<double> scale_tar_sour_list;
    std::vector<std::shared_ptr<vm::Frame>> graph_tar_list;
    std::vector<std::shared_ptr<vm::Frame>> graph_sour_list;
    
    
    std::vector<int> map_graph_v1;
    std::vector<int> map_graph_v2;
    std::vector<Eigen::Matrix4d> map_graph_sim3;
    std::vector<double> map_graph_scale;
    for(int i=0; i<maps.size()-1; i++){
        for(int j=i+1; j<maps.size(); j++){
            Eigen::Matrix4d sim3_j_i;
            double scale_j_i;
            bool succ = doAMatch(kp_info_list, *maps[i], *maps[j], desc_index_files[j],FLAGS_map_root_addr, T_tar_sour_list, scale_tar_sour_list, graph_tar_list, graph_sour_list, sim3_j_i, scale_j_i);
            if(succ){
                map_graph_v1.push_back(i);
                map_graph_v2.push_back(j);
                map_graph_sim3.push_back(sim3_j_i);
                map_graph_scale.push_back(scale_j_i);
                std::cout<<"match succ!! "<<i<<"->"<<j<<":"<<scale_j_i<<std::endl;
            }
        }
    }
    std::vector<int> expended_list;
    for(int i=0; i<maps.size(); i++){
        expended_list.push_back(0);
    }
    std::vector<int> p_list;
    std::vector<int> c_list;
    std::vector<Eigen::Matrix4d> T_c_ps;
    ExpandANode(0,expended_list, map_graph_v1, map_graph_v2, p_list, c_list, T_c_ps, map_graph_sim3);
    for(int i=0; i<p_list.size(); i++){
        std::cout<<"debug: "<<p_list[i]<<"->"<<c_list[i]<<std::endl;
    }
    int cur_map=0;
    int safe_count=0;
    std::vector<Eigen::Matrix4d> pose_in;
    for(int i=0; i<maps.size(); i++){
        pose_in.push_back(Eigen::Matrix4d::Identity());
    }
    while(true){
        safe_count++;
        bool findInP=false;
        for(int i=0; i<p_list.size(); i++){
            if(p_list[i]==cur_map){
                std::cout<<p_list[i]<<"->"<<c_list[i]<<std::endl;
                pose_in[c_list[i]]=T_c_ps[i]*pose_in[p_list[i]];
                cur_map=c_list[i];
                findInP=true;
                break;
            }
        }
        if(findInP==false){
            break;
        }
        if(safe_count>100){
            break;
        }
    }
    std::vector<Eigen::Vector3d> gps_alin;
    std::vector<int> gps_inlers;
    std::vector<Eigen::Matrix4d> poses_out;
    for(int i=0; i<pose_in.size(); i++){
        gps_inlers.push_back(0);
        Eigen::Vector3d temp=Eigen::Vector3d::Zero();
        gps_alin.push_back(temp);
        pose_in[i]=pose_in[i].inverse();
        //pose_in[i]=Eigen::Matrix4d::Identity();
    }
    for(int i=0; i<map_graph_sim3.size(); i++){
        map_graph_sim3[i].block(0,0,3,3)=map_graph_sim3[i].block(0,0,3,3)/map_graph_scale[i];
    }
    OptimizerTool::optimize_sim3_graph(gps_alin, gps_inlers, poses_out, pose_in, map_graph_sim3, map_graph_scale, map_graph_v1, map_graph_v2, true);
    
    vm::VisualMap base_map=*maps[0];
    for(int j=1; j<maps.size(); j++){
        GpsConverter gps_conv1(base_map.gps_anchor(0), base_map.gps_anchor(1), false);
        GpsConverter gps_conv2(maps[j]->gps_anchor(0), maps[j]->gps_anchor(1), false);
        for(int i=0; i<maps[j]->frames.size(); i++){
            WGS84Corr latlon;
            gps_conv2.MapXYToLatLon(maps[j]->frames[i]->position.x(), maps[j]->frames[i]->position.y(), latlon);
            UTMCoor xy;
            gps_conv1.MapLatLonToXY(latlon.lat, latlon.log, xy);
            maps[j]->frames[i]->position.x()=xy.x;
            maps[j]->frames[i]->position.y()=xy.y;
        }
    }
    
    for(int j=1; j<maps.size(); j++){
        Eigen::Matrix4d T_1_t = poses_out[j];
        for(int i=0; i<maps[j]->frames.size(); i++){
            Eigen::Matrix4d pose_transformed_temp;
            Eigen::Matrix4d temp_pose=maps[j]->frames[i]->getPose();
            
            transformPoseUseSim3(T_1_t, temp_pose, pose_transformed_temp);
            maps[j]->frames[i]->setPose(pose_transformed_temp);
        }
        for(int i=0; i<maps[j]->mappoints.size(); i++){
            Eigen::Vector4d posi_homo;
            posi_homo.block(0,0,3,1)=maps[j]->mappoints[i]->position;
            posi_homo(3)=1;
            Eigen::Vector4d posi_gps_homo = T_1_t*posi_homo;
            maps[j]->mappoints[i]->position=posi_gps_homo.block(0,0,3,1);                       
        }
        simpleMerge(base_map, *maps[j]);
    }

    for(int i=0; i<T_tar_sour_list.size(); i++){
        Eigen::Matrix3d rot=T_tar_sour_list[i].block(0,0,3,3)/scale_tar_sour_list[i];
        Eigen::Vector3d posi=T_tar_sour_list[i].block(0,3,3,1);
        base_map.AddConnection(graph_sour_list[i], graph_tar_list[i], posi,rot, scale_tar_sour_list[i], 100);
    }
    base_map.ComputeUniqueId();
    base_map.CheckConsistence();
    std::vector<int> merged_mps;
    
    for(int i=0; i<base_map.mappoints.size(); i++){
        merged_mps.push_back(-1);
    }
    int merged_mp_count=0;
    for(int i=0; i<base_map.mappoints.size(); i++){
        if(merged_mps[i]!=-1){
            continue;
        }
        int track_size=base_map.mappoints[i]->track.size();
        for(int j=0; j<track_size; j++){
            int temp_ip_id=findInKPList(kp_info_list, base_map.mappoints[i]->track[j]);
            if(temp_ip_id!=-1){ 
                for(int k=0; k<kp_info_list[temp_ip_id].mps.size(); k++){
                    if(merged_mps[kp_info_list[temp_ip_id].mps[k]->id]!=-1){
                        continue;
                    }
                    if(base_map.mappoints[i]->id==kp_info_list[temp_ip_id].mps[k]->id){
                        continue;
                    }
                    MergeMP(base_map.mappoints[i], kp_info_list[temp_ip_id].mps[k]);
                    merged_mps[kp_info_list[temp_ip_id].mps[k]->id]=0;
                    merged_mp_count++;
                    merged_mps[i]=1;
                }
            }
        }
    }
    LOG(INFO)<<"merged mp count: "<<merged_mp_count;
    for(int i=base_map.mappoints.size()-1; i>=0; i--){
        if(merged_mps[i]==0){
            base_map.mappoints.erase(base_map.mappoints.begin()+i);
        }
    }
    base_map.ComputeUniqueId();
    base_map.AssignKpToMp();
    base_map.CheckConsistence();
    vm::save_visual_map(base_map, FLAGS_map_root_addr+"/chamo_out.map");

    return 0;
}