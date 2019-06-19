#include "visual_map/visual_map.h"

namespace vm{
    void VisualMap::ComputeUniqueId(){
        for(int i=0; i<frames.size(); i++){
            frames[i]->id=i;
        }
        for(int i=0; i<mappoints.size(); i++){
            mappoints[i]->id=i;
        }
    }
    
    
    void VisualMap::CreateSubMap(int startframe_id, int endframe_id, VisualMap& submap){
        ComputeUniqueId();
        std::vector<int> old_to_new_id_map;
        for(int i=0; i<frames.size(); i++){
            old_to_new_id_map.push_back(-1);
        }
        for(int i=startframe_id; i<endframe_id; i++){
            std::shared_ptr<vm::Frame> frame_p;
            frame_p.reset(new vm::Frame);
            *(frame_p)=*(frames[i]);
            //std::cout<<"frames[i]->obss.size(): "<<frames[i]->obss.size()<<std::endl;
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
                    std::shared_ptr<vm::MapPoint> mappoint_p;
                    mappoint_p.reset(new vm::MapPoint);
                    *(mappoint_p)=*(mappoints[i]);
                    submap.mappoints.push_back(mappoint_p);
                    if(new_frameid>=submap.frames.size()){
                        std::cout<<"[CreateSubMap][error]new_frameid: "<<new_frameid<<":"<<submap.frames.size()<<std::endl;
                        exit(0);
                    }
                    mappoint_p->track[j].frame=submap.frames[new_frameid];
                    if(mappoints[i]->track[j].kp_ind>=submap.frames[new_frameid]->obss.size()){
                        std::cout<<"[CreateSubMap][error]mappoints[i]->track[j].kp_ind: "<<submap.frames[new_frameid]->obss.size()<<std::endl;
                        exit(0);
                    }
                    submap.frames[new_frameid]->obss[mappoints[i]->track[j].kp_ind]=mappoint_p;
                }
            }
        }
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
}