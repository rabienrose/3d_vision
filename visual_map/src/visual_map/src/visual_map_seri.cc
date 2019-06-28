#include "visual_map/visual_map_seri.h"
#include "VisualMap.pb.h"

namespace vm{

    void save_visual_map(VisualMap& map, std::string file_addr){
        proto::VisualMap map_proto;
        std::map<double, int> frame_time_to_index;
        map_proto.set_anchor_x(map.gps_anchor.x());
        map_proto.set_anchor_y(map.gps_anchor.y());
        map_proto.set_anchor_z(map.gps_anchor.z());
        for(int i=0; i<map.pose_graph_e_posi.size(); i++){
            proto::Sim3* sim3_e = map_proto.add_pose_graph_e();
            sim3_e->set_x(map.pose_graph_e_posi[i].x());
            sim3_e->set_y(map.pose_graph_e_posi[i].y());
            sim3_e->set_z(map.pose_graph_e_posi[i].z());
            Eigen::Quaterniond rot_qua(map.pose_graph_e_rot[i]);
            sim3_e->set_qx(rot_qua.x());
            sim3_e->set_qy(rot_qua.y());
            sim3_e->set_qz(rot_qua.z());
            sim3_e->set_qw(rot_qua.w());
            sim3_e->set_scale(map.pose_graph_e_scale[i]);
            map_proto.add_pose_graph_v1(map.pose_graph_v1[i]->id);
            map_proto.add_pose_graph_v2(map.pose_graph_v2[i]->id);
        }
        
        for(int i=0; i<map.frames.size(); i++){
            std::shared_ptr<Frame> frame_p=map.frames[i];
            proto::Frame* frame_proto = map_proto.add_frames();
            frame_proto->set_img_name(frame_p->frame_file_name);
            frame_proto->set_timestamp(frame_p->time_stamp);
            frame_proto->set_fx(frame_p->fx);
            frame_proto->set_fy(frame_p->fy);
            frame_proto->set_cx(frame_p->cx);
            frame_proto->set_cy(frame_p->cy);
            frame_proto->set_k1(frame_p->k1);
            frame_proto->set_k2(frame_p->k2);
            frame_proto->set_p1(frame_p->p1);
            frame_proto->set_p2(frame_p->p2);
            frame_proto->set_w(frame_p->width);
            frame_proto->set_h(frame_p->height);
            proto::Pose* pose_proto=frame_proto->mutable_pose();
            pose_proto->set_x(frame_p->position.x());
            pose_proto->set_y(frame_p->position.y());
            pose_proto->set_z(frame_p->position.z());
            pose_proto->set_qw(frame_p->direction.w());
            pose_proto->set_qx(frame_p->direction.x());
            pose_proto->set_qy(frame_p->direction.y());
            pose_proto->set_qz(frame_p->direction.z());
            pose_proto->set_gps_x(frame_p->gps_position.x());
            pose_proto->set_gps_y(frame_p->gps_position.y());
            pose_proto->set_gps_z(frame_p->gps_position.z());
            pose_proto->set_gps_accu(frame_p->gps_accu);
            frame_time_to_index[frame_p->time_stamp]=map_proto.frames_size()-1;
            std::vector<cv::KeyPoint>& keypoints1=frame_p->kps;
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descriptors1 = frame_p->descriptors;

            for (int i=0; i<keypoints1.size(); i++){
                proto::KeyPoint* kp = frame_proto->add_kps();
                kp->set_u(keypoints1[i].pt.x);
                kp->set_v(keypoints1[i].pt.y);
                kp->set_obs(-1);
                kp->set_octave(keypoints1[i].octave);
                std::string temp_ss;
                if(i>=descriptors1.cols()){
                    std::cout<<"[save_visual_map]kp count large than desc cols!"<<std::endl;
                    exit(0);
                }
                for(int j=0; j<descriptors1.rows(); j++){
                    temp_ss.push_back(descriptors1(j, i));
                    //std::cout<<(int)descriptors1.at<unsigned char>(j, i)<<std::endl;
                }
                //std::cout<<temp_ss<<std::endl;
                kp->set_desc_byte(temp_ss);
            }
        }
        
        std::cout<<"[save_visual_map]finish process frames."<<std::endl;
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
        
        if(map.frames.size()!=map_proto.frames_size()){
            std::cout<<"[save_visual_map][error]frame count not equal!!!"<<map.frames.size()<<":"<<map_proto.frames_size()<<std::endl;
            exit(0);
        }
        std::cout<<"[save_visual_map]finish process mappoints."<<std::endl;
        for(int i=0; i<map.frames.size(); i++){
            std::shared_ptr<Frame> frame_p=map.frames[i];
            proto::Frame* frame_proto = map_proto.mutable_frames(i);
            for(int j=0; j<frame_p->obss.size(); j++){
                if(frame_p->obss[j]!=nullptr){
                    if(mappoint_to_index.count(frame_p->obss[j].get())!=0){
                        if(frame_proto->kps_size()<=j){
                            std::cout<<"[save_visual_map][error]kp count less that index!!!"<<frame_proto->kps_size()<<":"<<j<<std::endl;
                            exit(0);
                        }
                        frame_proto->mutable_kps(j)->set_obs(mappoint_to_index[frame_p->obss[j].get()]);
                    }
                }
            }
        }
        
        std::cout<<"[save_visual_map]finish process covisibility."<<std::endl;
        std::fstream output(file_addr.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
        if (!map_proto.SerializeToOstream(&output)) {
            std::cerr << "Failed to write map data." << std::endl;
            return;
        }
        std::cout<<"[save_visual_map]finish save to disk."<<std::endl;
        output.close();
    }

    void loader_visual_map(VisualMap& map, std::string file_addr){
        proto::VisualMap map_proto;
        std::fstream input(file_addr.c_str(), std::ios::in | std::ios::binary);

        if (!map_proto.ParseFromIstream(&input)) {
            std::cerr << "Failed to parse map data." << std::endl;
            return ;
        }
        map.gps_anchor(0)=map_proto.anchor_x();
        map.gps_anchor(1)=map_proto.anchor_y();
        map.gps_anchor(2)=map_proto.anchor_z();
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
            frame_p->gps_position.x()=frame_proto.pose().gps_x();
            frame_p->gps_position.y()=frame_proto.pose().gps_y();
            frame_p->gps_position.z()=frame_proto.pose().gps_z();
            frame_p->gps_accu=frame_proto.pose().gps_accu();
            frame_p->fx=frame_proto.fx();
            frame_p->fy=frame_proto.fy();
            frame_p->cx=frame_proto.cx();
            frame_p->cy=frame_proto.cy();
            frame_p->k1=frame_proto.k1();
            frame_p->k2=frame_proto.k2();
            frame_p->p1=frame_proto.p1();
            frame_p->p2=frame_proto.p2();
            frame_p->width=frame_proto.w();
            frame_p->height=frame_proto.h();
            
            if(frame_proto.kps_size()>=0){
                int desc_width=frame_proto.kps(0).desc_byte().size();
                frame_p->descriptors.resize(desc_width, frame_proto.kps_size());
                for(int j=0; j<frame_proto.kps_size(); j++){
                    proto::KeyPoint keypoint_proto = frame_proto.kps(j);
                    cv::KeyPoint kp;
                    kp.pt.x=keypoint_proto.u();
                    kp.pt.y=keypoint_proto.v();
                    kp.octave=keypoint_proto.octave();
                    frame_p->kps.push_back(kp);
                    for (int k=0; k<desc_width; k++){
                        frame_p->descriptors(k,j)=keypoint_proto.desc_byte()[k];
                    }
                    
                    frame_p->obss.push_back(nullptr);
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
                    if(frame_proto.kps(j).obs()>=map.mappoints.size()){
                        std::cout<<"[loader_visual_map][error]frame_proto.kps(j).obs()"<<std::endl;
                        exit(0);
                        
                    }
                    map.frames[i]->obss[j]=map.mappoints[frame_proto.kps(j).obs()];
                }
            }
        }
        
        for(int i=0; i<map_proto.pose_graph_e_size(); i++){
            const proto::Sim3& sim3_proto = map_proto.pose_graph_e(i);
            Eigen::Vector3d posi;
            posi.x()=sim3_proto.x();
            posi.y()=sim3_proto.y();
            posi.z()=sim3_proto.z();
            Eigen::Quaterniond rot_qua;
            rot_qua.x()=sim3_proto.qx();
            rot_qua.y()=sim3_proto.qy();
            rot_qua.z()=sim3_proto.qz();
            rot_qua.w()=sim3_proto.qw();
            Eigen::Matrix3d rot(rot_qua);
            map.pose_graph_e_posi.push_back(posi);
            map.pose_graph_e_rot.push_back(rot);
            map.pose_graph_e_scale.push_back(sim3_proto.scale());
            map.pose_graph_v1.push_back(map.frames[map_proto.pose_graph_v1(i)]);
            map.pose_graph_v2.push_back(map.frames[map_proto.pose_graph_v2(i)]);
        }
        
    }
}   