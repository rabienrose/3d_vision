#include "visual_map/visual_map.h"
#include <glog/logging.h>

namespace vm{
    void VisualMap::ComputeUniqueId(){
        for(int i=0; i<frames.size(); i++){
            frames[i]->id=i;
        }
        for(int i=0; i<mappoints.size(); i++){
            mappoints[i]->id=i;
        }
    }
    
    std::shared_ptr<vm::MapPoint> VisualMap::getMPById(int id){
        for(int i=0; i<mappoints.size(); i++){
            if(id==mappoints[i]->id){
                return mappoints[i];
            }
        }
        return nullptr;
    }
    
    void VisualMap::AddConnection(std::shared_ptr<Frame> v1, std::shared_ptr<Frame> v2, Eigen::Vector3d& posi, Eigen::Matrix3d& rot,
        double scale, double weight){
        pose_graph_v1.push_back(v1);
        pose_graph_v2.push_back(v2);
        pose_graph_weight.push_back(weight);
        pose_graph_e_scale.push_back(scale);
        pose_graph_e_posi.push_back(posi);
        pose_graph_e_rot.push_back(rot);
    }
    
    
    void VisualMap::CreateSubMap(int startframe_id, int endframe_id, VisualMap& submap){
        std::cout<<"[CreateSubMap]"<<startframe_id<<":"<<endframe_id<<std::endl;
        ComputeUniqueId();
        submap=*(this);
        submap.frames.clear();
        submap.mappoints.clear();
        std::vector<int> old_to_new_id_map;
        for(int i=0; i<frames.size(); i++){
            old_to_new_id_map.push_back(-1);
        }
        for(int i=startframe_id; i<endframe_id; i++){
            std::shared_ptr<vm::Frame> frame_p;
            frame_p.reset(new vm::Frame);
            *(frame_p)=*(frames[i]);
            for(int j=0; j<frame_p->obss.size(); j++){
                frame_p->obss[j]=nullptr;
            }
            submap.frames.push_back(frame_p);
            old_to_new_id_map[i]=submap.frames.size()-1;
        }
        for(int i=0; i<mappoints.size(); i++){
            for(int j=0; j<mappoints[i]->track.size(); j++){
                if(mappoints[i]->track[j].frame->id>=old_to_new_id_map.size()){
                    std::cout<<"[CreateSubMap][error]mappoints[i]->track[j].frame->id"<<std::endl;
                    exit(0);
                }
                int new_frameid=old_to_new_id_map[mappoints[i]->track[j].frame->id];
                if(new_frameid!=-1){
                    std::shared_ptr<vm::MapPoint> mappoint_p= submap.getMPById(mappoints[i]->id);
                    if(mappoint_p==nullptr){
                        mappoint_p.reset(new vm::MapPoint);
                        *(mappoint_p)=*(mappoints[i]);
                        mappoint_p->track.clear();
                        submap.mappoints.push_back(mappoint_p);
                    }
                    
                    if(new_frameid>=submap.frames.size()){
                        std::cout<<"[CreateSubMap][error]new_frameid: "<<new_frameid<<":"<<submap.frames.size()<<std::endl;
                        exit(0);
                    }
                    submap.frames[new_frameid]->obss[mappoints[i]->track[j].kp_ind]=mappoint_p;                
                }
            }
        }
        submap.AssignKpToMp();
    }
    
    void VisualMap::DelMappoint(int id){
        for(int i=0; i<mappoints.size(); i++){
            if(mappoints[i]->id==id){
                std::shared_ptr<MapPoint> mp_p = mappoints[i];
                for(int j=0; j<mp_p->track.size(); j++){
                    std::shared_ptr<Frame> frame_p = mp_p->track[j].frame;
                    for(int k=0; k<frame_p->obss.size(); k++){
                        if(frame_p->obss[k]->id==mp_p->id){
                            frame_p->obss.erase(frame_p->obss.begin()+k); 
                            break;
                        }
                    }
                }
            }
        }
    }
    
    void VisualMap::DelFrame(int id){
        
    }
    
    void VisualMap::GetMPPosiList(std::vector<Eigen::Vector3d>& mp_posis){
        mp_posis.resize(mappoints.size());
        for(int i=0; i<mappoints.size(); i++){
            mp_posis[i]=mappoints[i]->position;
        }
    }
    
    void VisualMap::AssignKpToMp(){
        for(int i=0; i<mappoints.size(); i++){
            mappoints[i]->track.clear();
        }
        for(int i=0; i<frames.size(); i++){
            for(int k=0; k<frames[i]->obss.size(); k++){
                if(frames[i]->obss[k]!=nullptr){
                    TrackItem temp_track;
                    temp_track.frame=frames[i];
                    temp_track.kp_ind= k;
                    frames[i]->obss[k]->track.push_back(temp_track);
                }
            }
        }
    }
    
    void VisualMap::CalPoseEdgeVal(){
        for (int i=0; i<pose_graph_v1.size(); i++){
            Eigen::Matrix4d rel_pose= pose_graph_v2[i]->getPose().inverse() *pose_graph_v1[i]->getPose();
        }
    }
    
    void VisualMap::CheckConsistence(){
        for(int i=0; i<mappoints.size(); i++){
            for(int j=0; j<mappoints[i]->track.size(); j++){
                std::shared_ptr<vm::Frame> frame_p = mappoints[i]->track[j].frame;
                if(frame_p==nullptr){
                    LOG(INFO)<<"frame_p==nullptr";
                }
                std::shared_ptr<vm::MapPoint> mp = frame_p->obss[mappoints[i]->track[j].kp_ind];
                if(mp==nullptr){
                    LOG(INFO)<<"mp==nullptr";
                }else{
                    if(mp->id!=mappoints[i]->id){
                        LOG(INFO)<<"mp->id!=mappoints[i]->id: "<<mp->id<<" : "<<mappoints[i]->id;
                    }
                }
            }       
        }
        
        for(int i=0; i<frames.size(); i++){
            for(int j=0; j<frames[i]->obss.size(); j++){
                if(frames[i]->obss[j]!=nullptr){
                    std::shared_ptr<vm::MapPoint> mp=frames[i]->obss[j];
                    bool find_one=false;
                    for(int k=0; k<mp->track.size(); k++){
                        if(mp->track[k].frame->id==frames[i]->id && mp->track[k].kp_ind==j){
                            find_one=true;
                        }
                    }
                    if(find_one==false){
                        LOG(INFO)<<"find_one==false";
                        frames[i]->obss[j]==nullptr;
                    }
                }
            }
        }
    }
    
    void VisualMap::CheckLowQuaMappoint(){
        
    }
    
    void VisualMap::CheckLowQuaFrame(){
        
    }
}