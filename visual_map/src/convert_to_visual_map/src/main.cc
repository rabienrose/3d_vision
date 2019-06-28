#include <string>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "visual_map/visual_map.h"
#include <math.h>
#include "visual_map/visual_map_seri.h"

#include "read_write_data_lib/read_write.h"

void findFramePoseByName(std::vector<std::string>& names, int& re_id, std::string query_name){
    re_id=-1;
    for(int i=0; i<names.size(); i++){
        if(names[i]==query_name){
            re_id=i;
            return;
        }
    }
    return;
}

void findAllKP(std::string frame_name, std::vector<std::string>& kp_framename, std::vector<int>& old_kp_id_out){
    for(int i=0; i<kp_framename.size(); i++){
        if(kp_framename[i]==frame_name){
            old_kp_id_out.push_back(i);
        }
    }
}

int main(int argc, char* argv[]) {
    std::string res_root=argv[1];
    
    std::string img_time_addr=res_root+"/image_time.txt";
    std::vector<double> img_timess;
    std::vector<std::string> img_names;
    CHAMO::read_img_time(img_time_addr, img_timess, img_names);
    std::cout<<"img_timess: "<<img_timess.size()<<std::endl;
    
    std::vector<Eigen::Matrix4d> traj_out;
    std::vector<std::string> frame_names;
    std::string traj_file_addr = res_root+"/traj.txt";
    CHAMO::read_traj_file(traj_file_addr, traj_out, frame_names);
    std::cout<<"traj_out: "<<traj_out.size()<<std::endl;
    
    std::string kp_addr=res_root+"/kps.txt";
    std::vector<Eigen::Vector2f> kp_uvs;
    std::vector<std::string> kp_framename;
    std::vector<int> kp_octoves;
    CHAMO::read_kp_info(kp_addr, kp_uvs, kp_framename, kp_octoves);
    std::cout<<"kp_uvs: "<<kp_uvs.size()<<std::endl;
    
    std::string track_addr=res_root+"/track.txt";
    std::vector<std::vector<int>> tracks;
    CHAMO::read_track_info(track_addr, tracks);
    std::cout<<"tracks: "<<tracks.size()<<std::endl;
    
    std::string desc_addr=res_root+"/desc.txt";
    std::vector<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>> descs;
    CHAMO::read_desc_eigen(desc_addr, descs);
    
    std::string posi_addr=res_root+"/posi.txt";
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    std::cout<<"mp_posis: "<<mp_posis.size()<<std::endl;
    
    std::string gps_alin_addr=res_root+"/gps_alin.txt";
    std::vector<int> gps_inliers;
    std::vector<float> gps_accus;
    std::vector<Eigen::Vector3d> gps_alins;
    CHAMO::read_gps_alin(gps_alin_addr, gps_alins, gps_inliers, gps_accus);
    std::cout<<"gps_alins: "<<gps_alins.size()<<std::endl;
    if(gps_alins.size()!=img_names.size()){
        std::cout<<"gps count not equal frame count!!!"<<std::endl;
        exit(0);
    }
    
    std::string gps_orth_addr=res_root+"/gps_orth.txt";
    std::vector<Eigen::Vector3d> gps_orths;
    std::vector<double> gps_times;
    std::vector<int> gps_covs;
    Eigen::Vector3d anchor_gps;
    CHAMO::read_gps_orth(gps_orth_addr, gps_orths, gps_times, gps_covs, anchor_gps);
    std::cout<<"gps_times: "<<gps_times.size()<<std::endl;
    
    std::string cam_addr=res_root+"/camera_config.txt";
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    CHAMO::read_cam_info(cam_addr, cam_inter, cam_distort, Tbc);
    std::string image_config_addr=res_root+"/image_conf.txt";
    int width;
    int height; 
    float desc_scale;
    int desc_level; 
    int desc_count;
    CHAMO::read_image_info(image_config_addr, width, height, desc_scale, desc_level, desc_count);
    
    std::vector<int> old_to_new_kp_map;
    std::vector<std::shared_ptr<vm::Frame>> old_to_new_frame_map;
    for(int j=0; j<kp_framename.size(); j++){
        old_to_new_kp_map.push_back(-1);
    }
    for(int j=0; j<kp_framename.size(); j++){
        old_to_new_frame_map.push_back(nullptr);
    }
    
    vm::VisualMap map;
    for(int i=0; i<traj_out.size(); i++){
        
        std::shared_ptr<vm::Frame> frame_p;
        frame_p.reset(new vm::Frame);
        int re_id;
        findFramePoseByName(img_names, re_id, frame_names[i]);
        if(re_id<0){
            std::cout<<"Can not find id!"<<std::endl;
            exit(0);
        }
        frame_p->fx=cam_inter(0,0);
        frame_p->fy=cam_inter(1,1);
        frame_p->cx=cam_inter(0,2);
        frame_p->cy=cam_inter(1,2);
        frame_p->k1=cam_distort(0);
        frame_p->k2=cam_distort(1);
        frame_p->p1=cam_distort(2);
        frame_p->p2=cam_distort(3);
        frame_p->width=width;
        frame_p->height=height;
        frame_p->frame_file_name=frame_names[i];
        frame_p->time_stamp=img_timess[re_id];
        frame_p->position=traj_out[i].block(0,3,3,1);
        Eigen::Matrix3d tempRot = traj_out[i].block(0,0,3,3);
        Eigen::Quaterniond temp_qua(tempRot);
        frame_p->direction=temp_qua;
        frame_p->gps_position = gps_alins[re_id];
        frame_p->gps_accu = gps_accus[re_id];
        std::vector<int> old_kp_id_out;
        findAllKP(frame_names[i], kp_framename, old_kp_id_out);
        int desc_width=descs[i].rows();
        int desc_count=old_kp_id_out.size();
        frame_p->descriptors.resize(desc_width, desc_count);
        
        for(int j=0; j<old_kp_id_out.size(); j++){
            int temp_old_kp_id=old_kp_id_out[j];
            cv::KeyPoint kp;
            kp.pt.x=kp_uvs[temp_old_kp_id](0);
            kp.pt.y=kp_uvs[temp_old_kp_id](1);
            kp.octave=kp_octoves[temp_old_kp_id];
            frame_p->kps.push_back(kp);
            frame_p->descriptors.block(0,j,desc_width, 1)=descs[temp_old_kp_id];
            frame_p->obss.push_back(nullptr);
            if(temp_old_kp_id>=old_to_new_kp_map.size()){
                std::cout<<"[convert_to_visual_mp][error]temp_old_kp_id>=old_to_new_kp_map.size()"<<std::endl;
                exit(0);
            }
            old_to_new_kp_map[temp_old_kp_id]=j;
            old_to_new_frame_map[temp_old_kp_id]=frame_p;
        }
        map.frames.push_back(frame_p);
    }
    int null_count=0;
    for(int i=0; i<old_to_new_frame_map.size(); i++){
        if(old_to_new_frame_map[i]==nullptr){
            null_count++;
        }
    }
    std::cout<<"null kp count: "<<null_count<<std::endl;
    
    for(int i=0; i<mp_posis.size(); i++){
        std::shared_ptr<vm::MapPoint> mappoint_p;
        mappoint_p.reset(new vm::MapPoint);
        mappoint_p->position=mp_posis[i];
        for(int j=0; j<tracks[i].size(); j++){
            vm::TrackItem track_item;
            if(tracks[i][j]>=old_to_new_frame_map.size()){
                std::cout<<"[convert_to_visual_mp][error]tracks[i][j]>=old_to_new_kp_map.size()"<<std::endl;
                exit(0);
            }
            track_item.kp_ind = old_to_new_kp_map[tracks[i][j]];
            
            track_item.frame = old_to_new_frame_map[tracks[i][j]];
            if(track_item.frame==nullptr){
                //std::cout<<"[convert_to_visual_mp][error]track_item.frame==nullptr)"<<std::endl;
                continue;
            }
            mappoint_p->track.push_back(track_item);
            
            if(track_item.kp_ind>=track_item.frame->obss.size()){
                std::cout<<"[convert_to_visual_mp][error]track_item.kp_ind>=track_item.frame->obss.size()"<<std::endl;
                exit(0);
            }
            track_item.frame->obss[track_item.kp_ind]=mappoint_p;
        }
        map.mappoints.push_back(mappoint_p);
    }
    
    map.gps_anchor = anchor_gps;
    
    std::cout<<"load map finished!"<<std::endl;
    
    vm::save_visual_map(map, res_root+"/chamo.map");

    return 0;
}