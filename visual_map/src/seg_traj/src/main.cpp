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
    show_mp_as_cloud(img_gpss, "test_gps_pose");
    std::cout<<"start segmentation"<<std::endl;
    int cur_frame_id=traj_out.size()-1;
    int last_frame_id=0;
    int max_seg_count=50000;
    std::vector<Eigen::Vector3d> pc_frame_transformed;
    std::vector<Eigen::Matrix4d> pose_transformed;
    pose_transformed.resize(traj_out.size());
    while(ros::ok()){
        std::vector<Eigen::Vector3d> pc_frame;
        std::vector<Eigen::Matrix4d> pc_poses;
        std::vector<Eigen::Vector3d> pc_gps;
        for(int i=last_frame_id+1; i<cur_frame_id; i++){
            std::string query_img_name=frame_names[i];
            int re_imgid;
            findFramePoseByName(img_names, re_imgid, query_img_name);
            if(re_imgid!=-1 && img_to_gps_ids[re_imgid]!=-1){
                pc_frame.push_back(traj_out[i].block(0,3,3,1));
                pc_poses.push_back(traj_out[i]);
                pc_gps.push_back(img_gpss[img_to_gps_ids[re_imgid]]);
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
            if(avg_err>0.1 || pc_frame_transformed_temp.size()>max_seg_count || cur_frame_id==traj_out.size()-1){
                //std::cout<<cur_frame_id<<std::endl;
                pc_frame_transformed.insert(pc_frame_transformed.begin(), pc_frame_transformed_temp.begin(), pc_frame_transformed_temp.end());
                for(int i=last_frame_id; i<cur_frame_id; i++){
                    Eigen::Matrix4d pose_transformed_temp;
                    transformPoseUseSim3(T12, scale_12,  traj_out[i], pose_transformed_temp);
                    pose_transformed[i]=pose_transformed_temp;
                }
                if(cur_frame_id==traj_out.size()-1){
                    Eigen::Matrix4d pose_transformed_temp;
                    transformPoseUseSim3(T12, scale_12,  traj_out[cur_frame_id], pose_transformed_temp);
                    pose_transformed[cur_frame_id]=pose_transformed_temp;
                }
                show_mp_as_cloud(pc_frame_transformed, "test_frame_transformed");
                last_frame_id=cur_frame_id;
            }
        }
        cur_frame_id++;
        if(cur_frame_id==traj_out.size()){
            break;
        }
    }
    std::cout<<"end segmentation"<<std::endl;
    std::ofstream f;
    std::string pose_out_addr=res_addr+"/traj_alin.txt";
    f.open(pose_out_addr.c_str());
    for(int i=0; i<pose_transformed.size(); i++){
        f<<frame_names[i]<<","<<i
        <<","<<pose_transformed[i](0,0)<<","<<pose_transformed[i](0,1)<<","<<pose_transformed[i](0,2)<<","<<pose_transformed[i](0,3)
        <<","<<pose_transformed[i](1,0)<<","<<pose_transformed[i](1,1)<<","<<pose_transformed[i](1,2)<<","<<pose_transformed[i](1,3)
        <<","<<pose_transformed[i](2,0)<<","<<pose_transformed[i](2,1)<<","<<pose_transformed[i](2,2)<<","<<pose_transformed[i](2,3)
        <<std::endl;
    }
    f.close();
    
    std::string gps_out_addr=res_addr+"/gps_alin.txt";
    f.open(gps_out_addr.c_str());
    for(int i=0; i<pose_transformed.size(); i++){
        std::string query_img_name = frame_names[i];
        int re_imgid;
        findFramePoseByName(img_names, re_imgid, query_img_name);
        int gps_id = img_to_gps_ids[re_imgid];
        if(gps_id==-1){
            f<<frame_names[i]<<","<<"-1"
            <<std::endl;
        }else{
            Eigen::Vector3d gps_temp= img_gpss[gps_id];
            f<<frame_names[i]<<","<<i
            <<","<<gps_temp(0)<<","<<gps_temp(1)<<","<<gps_temp(2)
            <<std::endl;
        }
    }
    f.close();
    ros::spin();

    return 0;
}