#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "read_write_data_lib/read_write.h"
#include "visualization/common-rviz-visualization.h"
#include "orb_slam_lib/sim3_match.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_double(err_thres, 0.1, "if the sim3 match err is large than this, the trajectory will be cut");
DEFINE_int32(max_seg_frame_count, 100000, "max count of frame in a segment");
DEFINE_string(map_addr, "", "Folder of the map file, also the place to save the new map file.");
DEFINE_string(map_name, "", "File name of map file.");

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
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

int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    
    vm::VisualMap map;
    vm::loader_visual_map(map, FLAGS_map_addr+"/"+FLAGS_map_name);

    std::cout<<"start segmentation"<<std::endl;
    int cur_frame_id=10;
    int last_frame_id=0;
    int max_seg_count=FLAGS_max_seg_frame_count;
    std::vector<vm::VisualMap> submaps;
    while(ros::ok()){
        std::vector<Eigen::Vector3d> pc_frame;
        std::vector<Eigen::Vector3d> pc_gps;
        for(int i=last_frame_id+1; i<cur_frame_id; i++){
            std::shared_ptr<vm::Frame> frame = map.frames[i];
            if(frame->gps_accu<9999){
                pc_frame.push_back(frame->position);
                pc_gps.push_back(frame->gps_position);
            }else{
                //std::cout<<"cannot find frame name in pose list: "<<std::endl;
            }
        }

        if(pc_gps.size()>10){
            double scale_12;
            Eigen::Matrix4d T12;
            orb_slam::ComputeSim3(pc_gps, pc_frame , T12, scale_12);
            
            float avg_err=0;
            std::vector<Eigen::Vector3d> pc_frame_transformed_temp;
            for(int i=0; i<pc_gps.size(); i++){
                Eigen::Vector4d posi_homo;
                posi_homo.block(0,0,3,1)=pc_frame[i];
                posi_homo(3)=1;
                Eigen::Vector4d posi_gps_homo = T12*posi_homo;
                float err = (pc_gps[i]-posi_gps_homo.block(0,0,3,1)).norm();
                avg_err=err/pc_gps.size();
                pc_frame_transformed_temp.push_back(posi_gps_homo.block(0,0,3,1));
            }
            //std::cout<<"avg_err: "<<cur_frame_id<<" | "<<pc_gps.size()<<" | "<<avg_err<<std::endl;
            if(avg_err>FLAGS_err_thres || pc_frame_transformed_temp.size()>max_seg_count || cur_frame_id==map.frames.size()-1){
                std::cout<<"[seg_traj]start new seg: "<<submaps.size()<<std::endl;
                int extend_start=last_frame_id-10;
                if(extend_start<0){
                    extend_start=0;
                }
                int extend_end=cur_frame_id+10;
                if(extend_end>=map.frames.size()){
                    extend_end=map.frames.size();
                }
                vm::VisualMap submap;
                map.CreateSubMap(extend_start, extend_end, submap); //not include extend_end
                for(int i=extend_start; i<extend_end; i++){
                    Eigen::Matrix4d pose_transformed_temp;
                    Eigen::Matrix4d temp_pose=map.frames[i]->getPose();
                    transformPoseUseSim3(T12, scale_12, temp_pose, pose_transformed_temp);
                    if(i-extend_start>=submap.frames.size()){
                        std::cout<<"[seg_traj][error]i-extend_start>=submap.frames.size()"<<std::endl;
                    }
                    submap.frames[i-extend_start]->setPose(pose_transformed_temp);
                }
                for(int j=0; j<submap.mappoints.size(); j++){
                    Eigen::Vector4d posi_homo;
                    posi_homo.block(0,0,3,1)=submap.mappoints[j]->position;
                    posi_homo(3)=1;
                    Eigen::Vector4d posi_gps_homo = T12*posi_homo;
                    submap.mappoints[j]->position=posi_gps_homo.block(0,0,3,1);                       
                }
                submaps.push_back(submap);
                last_frame_id=cur_frame_id;
            }
        }
        cur_frame_id++;
        if(cur_frame_id==map.frames.size()){
            break;
        }
    }
    std::cout<<"end segmentation"<<std::endl;
    
    for(int i=0; i<submaps.size(); i++){
        std::stringstream ss;
        ss<<"/"<<1000+i<<"_"<<FLAGS_map_name;
        submaps[i].UpdatePoseEdge();
        vm::save_visual_map(submaps[i], FLAGS_map_addr+ss.str());
    }
    return 0;
}