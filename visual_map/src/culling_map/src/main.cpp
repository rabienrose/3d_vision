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
DEFINE_double(search_range, 50, "Range of size to consider as candidator of compare with current KF.");

void findNNFrames(std::vector<std::shared_ptr<vm::Frame>>& inputs, std::vector<std::shared_ptr<vm::Frame>>& outputs, std::shared_ptr<vm::Frame> query){
    for(int i=0; i<inputs.size(); i++){
        if((inputs[i]->position-query->position).norm()<FLAGS_search_range){
            outputs.push_back(inputs[i]);
        }
    }
}

int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    vm::VisualMap map;
    std::cout<<FLAGS_map_addr+"/"+FLAGS_map_name<<std::endl;
    vm::loader_visual_map(map, FLAGS_map_addr+"/"+FLAGS_map_name);
    int mp_count=0;
    
    double t_proj_err=0;
    
    std::vector<int> del_mp_ids;
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
                if(proj_err>5){
                    map.frames[i]->obss[j]=nullptr;
                }
            }
        }
    }
    map.AssignKpToMp();
    map.ComputeUniqueId();
    int total_size=map.mappoints.size();
    for(int i=total_size-1; i>=0; i--){
        if(map.mappoints[i]->track.size()<2){
            map.DelMappoint(map.mappoints[i]->id);
            mp_count++;
        }
    }
    LOG(INFO)<<"del "<<mp_count<<" mp!"<<std::endl;
    map.AssignKpToMp();
    map.ComputeUniqueId();
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
            for(int j=0 ; j<map.frames[i]->obss.size(); j++){
                if(map.frames[i]->obss[j]!=nullptr){
                    for(int k=0; k<map.frames[i]->obss[j]->track.size(); k++){
                        std::shared_ptr<vm::Frame> temp_frame_p = map.frames[i]->obss[j]->track[k].frame;
                        CHECK_NOTNULL(temp_frame_p);
                        if(temp_frame_p->id==-1){
                            continue;
                        }
                        if(temp_frame_p->id==map.frames[i]->id){
                            continue;
                        }
                        if(frame_list.count(temp_frame_p)==0){
                            frame_list[temp_frame_p]=1;
                        }else{
                            frame_list[temp_frame_p]=frame_list[temp_frame_p]+1;
                        }
                    }
                }
            }
            std::map<std::shared_ptr<vm::Frame>, int>::iterator it;
            
            for ( it = frame_list.begin(); it != frame_list.end(); it++ ){
                float rate_1=it->second/(float)frame_obss_count[map.frames[i]->id];
                float rate_2=it->second/(float)frame_obss_count[it->first->id];
                
                if(rate_1>0.6){
                    if(rate_2>0.6){
                        //std::cout<<rate_1<<":"<<rate_2<<std::endl;
                        //std::cout<<"del :"<<it->first->id<<std::endl;
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
    
    map.ComputeUniqueId();
    vm::save_visual_map(map, FLAGS_map_addr+"/culling_"+FLAGS_map_name);
    return 0;
}