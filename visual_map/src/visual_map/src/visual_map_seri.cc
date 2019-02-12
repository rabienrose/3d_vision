#include "visual_map/visual_map_seri.h"
#include "VisualMap.pb.h"

namespace vm{

    void save_visual_map(VisualMap& map, std::string file_addr){
        proto::VisualMap map_proto;
        std::map<double, int> frame_time_to_index;
        for(int i=0; i<map.frames.size(); i++){
            std::shared_ptr<Frame> frame_p=map.frames[i];
            proto::Frame* frame_proto = map_proto.add_frames();
            frame_proto->set_img_name(frame_p->frame_file_name);
            frame_proto->set_timestamp(frame_p->time_stamp);
            proto::Pose* pose_proto=frame_proto->mutable_pose();
            pose_proto->set_x(frame_p->position.x());
            pose_proto->set_y(frame_p->position.y());
            pose_proto->set_z(frame_p->position.z());
            pose_proto->set_qw(frame_p->direction.w());
            pose_proto->set_qx(frame_p->direction.x());
            pose_proto->set_qy(frame_p->direction.y());
            pose_proto->set_qz(frame_p->direction.z());
            frame_time_to_index[frame_p->time_stamp]=map_proto.frames_size()-1;
            std::vector<cv::KeyPoint>& keypoints1=frame_p->kps;
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descriptors1 = frame_p->descriptors;
            for (int i=0; i<keypoints1.size(); i++){
                proto::KeyPoint* kp = frame_proto->add_kps();
                kp->set_u(keypoints1[i].pt.x);
                kp->set_v(keypoints1[i].pt.y);
                kp->set_obs(-1);
                std::string temp_ss;
                for(int j=0; j<descriptors1.cols(); j++){
                    temp_ss.push_back(descriptors1(i, j));
                    //std::cout<<(int)descriptors1.at<unsigned char>(j, i)<<std::endl;
                }
                //std::cout<<temp_ss<<std::endl;
                kp->set_desc_byte(temp_ss);
            }
        }
        std::map<MapPoint*, int> mappoint_to_index;
        for(int i=0; i<map.mappoints.size(); i++){
            std::shared_ptr<MapPoint> mappoint_p = map.mappoints[i];
            proto::MapPoint* mappoint_proto = map_proto.add_mappoints();
            mappoint_to_index[mappoint_p.get()]=map_proto.mappoints_size()-1;
            mappoint_proto->set_x(mappoint_p->position.x());
            mappoint_proto->set_y(mappoint_p->position.y());
            mappoint_proto->set_z(mappoint_p->position.z());
            for(int j=0; j<mappoint_p->track.size(); j++){
                TrackItem trach_item = mappoint_p->track[j];
                if(frame_time_to_index.count(trach_item.frame->time_stamp)!=0){
                    int frame_ind = frame_time_to_index[trach_item.frame->time_stamp];
                    proto::Track* track_proto = mappoint_proto->add_tracks();
                    track_proto->set_frame_id(frame_ind);
                    track_proto->set_kp_id(trach_item.kp_ind);
                }
            }
        }
        for(int i=0; i<map.frames.size(); i++){
            std::shared_ptr<Frame> frame_p=map.frames[i];
            proto::Frame frame_proto = map_proto.frames(i);
            for(int j=0; j<frame_p->obss.size(); j++){
                if(frame_p->obss[j]!=nullptr){
                    if(mappoint_to_index.count(frame_p->obss[j].get())!=0){
                        frame_proto.mutable_kps(j)->set_obs(mappoint_to_index[frame_p->obss[j].get()]);
                    }
                }
            }
        }
        
        std::fstream output(file_addr.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
        if (!map_proto.SerializeToOstream(&output)) {
            std::cerr << "Failed to write map data." << std::endl;
            return;
        }
        
        output.close();
    }

    void loader_visual_map(VisualMap& map, std::string file_addr){
        proto::VisualMap map_proto;
        std::fstream input(file_addr.c_str(), std::ios::in | std::ios::binary);

        if (!map_proto.ParseFromIstream(&input)) {
            std::cerr << "Failed to parse map data." << std::endl;
            return ;
        }
        
        for(int i=0; i<map_proto.frames_size(); i++){
            const proto::Frame& frame_proto = map_proto.frames(i);
            std::shared_ptr<vm::Frame> frame_p;
            frame_p.reset(new vm::Frame);
            frame_p->frame_file_name=frame_proto.img_name();
            frame_p->time_stamp=frame_proto.timestamp();
            frame_p->position.x()=frame_proto.pose().x();
            frame_p->position.y()=frame_proto.pose().y();
            frame_p->position.z()=frame_proto.pose().z();
            frame_p->direction.w()=frame_proto.pose().qw();
            frame_p->direction.x()=frame_proto.pose().qx();
            frame_p->direction.y()=frame_proto.pose().qy();
            frame_p->direction.z()=frame_proto.pose().qz();
            if(frame_proto.kps_size()>=0){
                int desc_width=frame_proto.kps(0).desc_byte().size();
                frame_p->descriptors.resize(frame_proto.kps_size(), desc_width);
                for(int j=0; j<frame_proto.kps_size(); j++){
                    proto::KeyPoint keypoint_proto = frame_proto.kps(j);
                    cv::KeyPoint kp;
                    kp.pt.x=keypoint_proto.u();
                    kp.pt.y=keypoint_proto.v();
                    frame_p->kps.push_back(kp);
                    for (int k=0; k<desc_width; k++){
                        frame_p->descriptors(j,k)=keypoint_proto.desc_byte()[k];
                    }
                }
            }
            map.frames.push_back(frame_p);
        }
        
        for(int i=0; i<map_proto.mappoints_size(); i++){
            const proto::MapPoint& mappoint_proto = map_proto.mappoints(i);
            std::shared_ptr<vm::MapPoint> mappoint_p;
            mappoint_p.reset(new vm::MapPoint);
            mappoint_p->position.x()=mappoint_proto.x();
            mappoint_p->position.y()=mappoint_proto.y();
            mappoint_p->position.z()=mappoint_proto.z();
            for(int j=0; j<mappoint_proto.tracks_size(); j++){
                TrackItem item;
                item.frame = map.frames[mappoint_proto.tracks(j).frame_id()];
                item.kp_ind=mappoint_proto.tracks(j).kp_id();
                mappoint_p->track.push_back(item);
            }
            map.mappoints.push_back(mappoint_p);
        }
        
        for(int i=0; i<map_proto.frames_size(); i++){
            const proto::Frame& frame_proto = map_proto.frames(i);
            for (int j=0; j<frame_proto.kps_size(); j++){
                if(frame_proto.kps(j).obs()!=-1){
                    map.frames[i]->obss.push_back(map.mappoints[frame_proto.kps(j).obs()]);
                }
            }
        }
    }
}   