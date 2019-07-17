#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "read_write_data_lib/read_write.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(map_addr, "", "Folder of the map file, also the place to save the new map file.");
DEFINE_string(map_name, "", "File name of map file.");
DEFINE_string(culling_type, "", "mp | frame | project.");
DEFINE_double(max_repro_err, 5, "");
DEFINE_double(max_proj_desc_err, 40, "");

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
    
    vm::VisualMap map;
    std::cout<<FLAGS_map_addr+"/"+FLAGS_map_name<<std::endl;
    vm::loader_visual_map(map, FLAGS_map_addr+"/"+FLAGS_map_name);
    
    if(FLAGS_culling_type=="mp"){
        int del_edge_count=0;
        for(int i=0; i<map.frames.size(); i++){
            for(int j=0; j<map.frames[i]->obss.size(); j++){
                if(map.frames[i]->obss[j]!=nullptr){
                    Eigen::Matrix<double, 3,4> proj_mat = map.frames[i]->getProjMat();
                    Eigen::Vector4d posi_homo;
                    posi_homo.block(0,0,3,1)=map.frames[i]->obss[j]->position;
                    posi_homo(3)=1;
                    Eigen::Vector3d proj_homo = proj_mat*posi_homo;
                    //std::cout<<proj_mat<<std::endl;
                    double u=proj_homo(0)/proj_homo(2);
                    double v=proj_homo(1)/proj_homo(2);
                    cv::Point2f uv= map.frames[i]->kps[j].pt;
                    //std::cout<<u<<":"<<v<<"     "<<uv.x<<":"<<uv.y<<std::endl;
                    
                    float proj_err=sqrt((uv.x-u)*(uv.x-u)+(uv.y-v)*(uv.y-v));
                    if(proj_err>FLAGS_max_repro_err){
                        del_edge_count++;
                        map.frames[i]->obss[j]=nullptr;
                    }
                }
            }
        }
        int mp_count=0;
        map.AssignKpToMp();
        map.ComputeUniqueId();
        int total_size=map.mappoints.size();
        for(int i=total_size-1; i>=0; i--){
            if(map.mappoints[i]->track.size()<2){
                map.DelMappoint(map.mappoints[i]->id);
                mp_count++;
            }
        }
        LOG(INFO)<<"del "<<del_edge_count<<" edges!"<<std::endl;
        LOG(INFO)<<"del "<<mp_count<<" mp!"<<std::endl;
    }else if(FLAGS_culling_type=="frame"){
        map.AssignKpToMp();
        map.ComputeUniqueId();
        map.FilterTrack();
        map.AssignKpToMp();
        bool del_any_frame=false;
        std::vector<int> frame_obss_count;
        for(int i=0; i<map.frames.size(); i++){
            int cont_t=0;
            for(int j=0; j<map.frames[i]->obss.size(); j++){
                if(map.frames[i]->obss[j]!=nullptr){
                    cont_t++;
                }
            }
            frame_obss_count.push_back(cont_t);
        }
        do{
            del_any_frame=false;
            for(int i=0; i<map.frames.size(); i++){
                std::map<std::shared_ptr<vm::Frame>, int> frame_list;
                map.GetCovisi(map.frames[i], frame_list);
                std::map<std::shared_ptr<vm::Frame>, int>::iterator it;
                for ( it = frame_list.begin(); it != frame_list.end(); it++ ){
                    CHECK_NE(map.frames[i]->id, -1);
                    CHECK_NE(it->first->id, -1);
                    float rate_1=it->second/(float)frame_obss_count[map.frames[i]->id];
                    float rate_2=it->second/(float)frame_obss_count[it->first->id];
                    
                    if(rate_1>0.6){
                        if(rate_2>0.6){
                            //std::cout<<rate_1<<":"<<rate_2<<std::endl;
                            //std::cout<<"del :"<<it->first->id<<std::endl;
                            //std::cout<<"it->second :"<<it->second<<std::endl;
                            int cont_t=0;
                            for(int j=0; j<map.frames[i]->obss.size(); j++){
                                if(map.frames[i]->obss[j]!=nullptr){
                                    cont_t++;
                                }
                            }
                            //std::cout<<"cont_t :"<<cont_t<<std::endl;
                            map.DelFrame(it->first->id);
                            del_any_frame=true;
                            break;
                        }
                    }
                }
                if(del_any_frame){
                    break;
                }
            }
        }while(del_any_frame==true);
    }else if(FLAGS_culling_type=="project"){
        for(int i=0; i<map.frames.size(); i++){
            for(int j=0; j<map.mappoints.size(); j++){
                Eigen::Matrix<double, 3,4> proj_mat = map.frames[i]->getProjMat();
                Eigen::Vector4d posi_homo;
                posi_homo.block(0,0,3,1)=map.mappoints[j]->position;
                posi_homo(3)=1;
                Eigen::Vector3d proj_homo = proj_mat*posi_homo;
                //std::cout<<proj_mat<<std::endl;
                double u=proj_homo(0)/proj_homo(2);
                double v=proj_homo(1)/proj_homo(2);
                for(int k=0; k<map.frames[i]->kps.size(); k++){
                    cv::Point2f uv= map.frames[i]->kps[k].pt;                
                    float proj_err=sqrt((uv.x-u)*(uv.x-u)+(uv.y-v)*(uv.y-v));
                    if(proj_err<5){
                        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> query_desc = map.frames[i]->descriptors.col(k);
                        int diff = map.mappoints[j]->calDescDiff(query_desc);
                        //std::cout<<diff<<std::endl;
                        if(diff<FLAGS_max_proj_desc_err){
                            
                            if(map.frames[i]->obss[k]==nullptr){
                                std::cout<<"add: "<<i<<":"<<j<<":"<<k<<":"<<diff<<std::endl;
                                map.frames[i]->obss[k]=map.mappoints[j];
                            }else{
                                if(map.mappoints[j]->id!=map.frames[i]->obss[k]->id){
                                    std::cout<<"merge: "<<i<<":"<<j<<":"<<k<<":"<<diff<<std::endl;
                                    MergeMP(map.mappoints[j], map.frames[i]->obss[k]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }else if(FLAGS_culling_type=="stable"){
        int total_size=map.mappoints.size();
        map.ComputeUniqueId();
        for(int i=total_size-1; i>=0; i--){
            if(map.mappoints[i]->match_count==0){
                map.DelMappoint(map.mappoints[i]->id);
            }
        }
    }
    vm::save_visual_map(map, FLAGS_map_addr+"/culling_"+FLAGS_map_name);
    return 0;
}