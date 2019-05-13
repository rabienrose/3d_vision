#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "read_write_data_lib/read_write.h"
#include "visualization/common-rviz-visualization.h"
#include "orb_slam_lib/sim3_match.h"

void findNearGPS(int& gps1, int& gps2, std::vector<double>& gps_times, double frame_time){
    gps1=-1;
    gps2=-1;
    for(int i=0; i<gps_times.size() ; i++){
        if(gps_times[i]>frame_time){
            if(i==0){
                return;
            }
            if(frame_time - gps_times[i-1]>2){
                return;
            }
            if(gps_times[i] - frame_time >2){
                return;
            }
            gps1=i-1;
            gps2=i;
            return;
        }
    }
}

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

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

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_addr=argv[1];
    std::string img_time_addr=res_addr+"/image_time.txt";
    std::vector<double> img_timess;
    std::vector<std::string> img_names;
    CHAMO::read_img_time(img_time_addr, img_timess, img_names);
    std::cout<<"img_timess: "<<img_timess.size()<<std::endl;
    
    std::string gps_orth_addr = res_addr+"/gps_orth.txt";
    std::vector<Eigen::Vector3d> gps_orths;
    std::vector<double> gps_times;
    std::vector<int> gps_covs;
    Eigen::Vector3d anchor_gps;
    CHAMO::read_gps_orth(gps_orth_addr, gps_orths, gps_times, gps_covs, anchor_gps);
    std::cout<<"gps_orths: "<<gps_orths.size()<<std::endl;
    
    std::vector<Eigen::Matrix4d> traj_out;
    std::vector<std::string> frame_names;
    std::string traj_file_addr = res_addr+"/traj.txt";
    CHAMO::read_traj_file(traj_file_addr, traj_out, frame_names);
    std::cout<<"traj_out: "<<traj_out.size()<<std::endl;
    std::vector<double> gps_times_hcov;
    std::vector<Eigen::Vector3d> gps_orths_hcov;
    for (int i=0; i<gps_covs.size(); i++){
        if(gps_covs[i]<10){
            gps_times_hcov.push_back(gps_times[i]);
            gps_orths_hcov.push_back(gps_orths[i]);
        }
    }
    std::vector<int> img_to_gps_ids;
    std::vector<int> gps_to_img_ids;
    std::vector<Eigen::Vector3d> img_gpss;
    for (int i=0; i<img_timess.size(); i++){
        int gps1;
        int gps2;
        findNearGPS(gps1, gps2, gps_times_hcov, img_timess[i]);
        
        if(gps1== -1){
            img_to_gps_ids.push_back(-1);
            continue;
        }
        double i_gps_x;
        double i_gps_y;
        double i_gps_z;
        interDouble(gps_orths_hcov[gps1](0), gps_orths_hcov[gps2](0), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_x, img_timess[i]);
        interDouble(gps_orths_hcov[gps1](1), gps_orths_hcov[gps2](1), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_y, img_timess[i]);
        interDouble(gps_orths_hcov[gps1](2), gps_orths_hcov[gps2](2), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_z, img_timess[i]);
        Eigen::Vector3d new_gps_frame;
        new_gps_frame(0)=i_gps_x;
        new_gps_frame(1)=i_gps_y;
        new_gps_frame(2)=i_gps_z;
        img_gpss.push_back(new_gps_frame);
        img_to_gps_ids.push_back(img_gpss.size()-1);
        gps_to_img_ids.push_back(i);
    }
    
    std::cout<<"start segmentation"<<std::endl;
    int cur_frame_id=3;
    int last_frame_id=0;
    while(ros::ok()){
        std::vector<Eigen::Vector3d> pc_frame;
        std::vector<Eigen::Vector3d> pc_gps;
        for(int i=last_frame_id+1; i<cur_frame_id; i++){
            std::string query_img_name=frame_names[i];
            int re_imgid;
            findFramePoseByName(img_names, re_imgid, query_img_name);
            if(re_imgid!=-1 && img_to_gps_ids[re_imgid]!=-1){
                pc_frame.push_back(traj_out[i].block(0,3,3,1));
                pc_gps.push_back(img_gpss[img_to_gps_ids[re_imgid]]);
            }else{
                //std::cout<<"cannot find frame name in pose list"<<std::endl;
            }
        }
        if(pc_gps.size()>5){
            Eigen::Matrix4d T12;
            orb_slam::ComputeSim3(pc_gps, pc_frame , T12);
            std::vector<Eigen::Vector3d> pc_frame_transformed;
            float avg_err=0;
            for(int i=0; i<pc_gps.size(); i++){
                Eigen::Vector4d posi_homo;
                posi_homo.block(0,0,3,1)=pc_frame[i];
                posi_homo(3)=1;
                Eigen::Vector4d posi_gps_homo = T12*posi_homo;
                float err = (pc_gps[i]-posi_gps_homo.block(0,0,3,1)).norm();
                avg_err=err/pc_gps.size();
                pc_frame_transformed.push_back(posi_gps_homo.block(0,0,3,1));
                show_mp_as_cloud(pc_gps, "test_gps_pose");
                show_mp_as_cloud(pc_frame_transformed, "test_frame_transformed");
            }
            std::cout<<"avg_err: "<<cur_frame_id<<" | "<<pc_gps.size()<<" | "<<avg_err<<std::endl;
        }
        cur_frame_id++;
        if(cur_frame_id>=traj_out.size()){
            break;
        }
    }
    
//     for(int i=0; i<pc_frame.size(); i++){
//         Eigen::Vector4d posi_homo;
//         posi_homo.block(0,0,3,1)=pc_frame[i];
//         posi_homo(3)=1;
//         Eigen::Vector4d posi_gps_homo = T12*posi_homo;
//         pc_frame_transformed.push_back(posi_gps_homo.block(0,0,3,1));
//     }
    
    //show_mp_as_cloud(pc_frame, "test_frame_pose");
    //show_mp_as_cloud(pc_gps, "test_gps_pose");
    //show_mp_as_cloud(pc_frame_transformed, "test_frame_transformed");
    ros::spin();

    return 0;
}