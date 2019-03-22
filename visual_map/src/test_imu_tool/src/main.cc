#include <string>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <math.h>
#include "imu_tools.h"
#include "NavState.h"
#include "Converter.h"

#include "test_imu_tool/visual_tool.h"
#include "read_write_data_lib/read_write.h"
#include "visualization/common-rviz-visualization.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"


Eigen::Vector3d interEigenV(Eigen::Vector3d v1, Eigen::Vector3d v2, double t1, double t2, double t3){
    return v1 + (v2 - v1) * (t3 - t1) / (t2 - t1);
}

void updatePreInt(std::vector<ORB_SLAM2::IMUPreintegrator>& preints, std::vector<std::vector<ORB_SLAM2::IMUData>>& sycn_imu_datas,
    Eigen::Vector3d ba, Eigen::Vector3d bg
){
    for(int i=0; i<sycn_imu_datas.size(); i++){
        ORB_SLAM2::IMUPreintegrator temp_preint;
        temp_preint.reset();
        for(int j=1; j<sycn_imu_datas[i].size(); j++){
            double dt=sycn_imu_datas[i][j]._t-sycn_imu_datas[i][j-1]._t;
            temp_preint.update(sycn_imu_datas[i][j]._g - bg, sycn_imu_datas[i][j]._a - ba, dt);
        }
        //std::cout<<temp_preint.getDeltaP().transpose()<<std::endl;
        preints.push_back(temp_preint);
    }
}

int main(int argc, char* argv[]) {
    std::cout<<"chamo"<<std::endl;
    visualization::RVizVisualizationSink::init();
    std::string save_addr;
    std::string line;
    
    Eigen::Matrix4d Tbc= Eigen::Matrix4d::Identity();
    Tbc<<-0.99999518, -0.00310315,  0.00007742,  0.05310734,
        -0.00124823,  0.42483127,  0.90527169,  0.00929943,
        -0.00284208,  0.90526723, -0.4248331,  -0.01557671,
        0.0,          0.0,          0.0,         1.0;
    float fx=541.39699791; 
    float fy=540.86222539; 
    float cx=474.6630494; 
    float cy=306.8632145; 
    
    std::string img_time_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/camera_1_image_time.txt";
    std::string pose_addr="/home/chamo/Documents/work/orb_mapping_loc/traj.txt";
    std::map<double, int> pose_list;
    std::map<int, int> frame_ids;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> pose_vec;
    CHAMO::read_pose_list(pose_list, frame_ids, pose_vec, pose_addr, img_time_addr);

    std::string imu_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/imu.txt";
    std::vector<Eigen::Matrix<double, 7, 1>> imu_datas_raw;
    CHAMO::read_imu_data(imu_addr, imu_datas_raw);

    std::string posi_addr="/home/chamo/Documents/work/orb_mapping_loc/posi.txt";
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    
    std::string kp_addr="/home/chamo/Documents/work/orb_mapping_loc/kps.txt";
    std::vector<Eigen::Vector2f> kp_uvs;
    std::vector<int> kp_frameids;
    std::vector<int> kp_octoves;
    CHAMO::read_kp_info(kp_addr, kp_uvs, kp_frameids, kp_octoves);
    
    std::string track_addr="/home/chamo/Documents/work/orb_mapping_loc/track.txt";
    std::vector<std::vector<int>> tracks;
    CHAMO::read_track_info(track_addr, tracks);
    
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lidar_posis;
    std::vector<Eigen::Quaterniond> lidar_dirs;
    std::string lidar_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/wayz_2018_11_26_result/interpolation_traj_camera_right.txt";
    CHAMO::read_lidar_pose(lidar_addr, lidar_dirs, lidar_posis);
    
    std::vector<std::vector<ORB_SLAM2::MP_INFO>> mp_infos;
    for(int i=0; i<tracks.size(); i++){
        std::vector<ORB_SLAM2::MP_INFO> track_info;
        for(int j=0; j<tracks[i].size(); j++){
            int kp_id=tracks[i][j];
            ORB_SLAM2::MP_INFO info;
            info.u=kp_uvs[kp_id].x();
            info.v=kp_uvs[kp_id].y();
            info.octove=kp_octoves[kp_id];
            info.frame_id=frame_ids[kp_frameids[kp_id]];
            info.mp_id=i;
            track_info.push_back(info);
        }
        mp_infos.push_back(track_info);
    }
    
    std::vector<ORB_SLAM2::IMUData> imu_datas;
    for(int i=0; i<imu_datas_raw.size(); i++){
        double timestamp=imu_datas_raw[i](0);
        double gx=imu_datas_raw[i](1);
        double gy=imu_datas_raw[i](2);
        double gz=imu_datas_raw[i](3);
        double ax=imu_datas_raw[i](4);
        double ay=imu_datas_raw[i](5);
        double az=imu_datas_raw[i](6);
        ORB_SLAM2::IMUData imu_data(gx, gy, gz, ax, ay, az, timestamp);
        imu_datas.push_back(imu_data);
    }

    std::vector<std::vector<ORB_SLAM2::IMUData>> sycn_imu_datas_all;
    int procceing_imu_id=0;
    double last_time=0;
    for(auto item: pose_list){
        double time = item.first;
        std::vector<ORB_SLAM2::IMUData> temp_imu_dat;
        ORB_SLAM2::IMUData imu_data_temp;
        imu_data_temp._t=last_time; //first data is not used, whatever
        last_time=time;
        temp_imu_dat.push_back(imu_data_temp); //first data for one frame is not used, except the timestamp
        while(true){
            int i=procceing_imu_id;
            if(imu_datas[i]._t>time){
                ORB_SLAM2::IMUData imu_data_temp;
                imu_data_temp._g = interEigenV(imu_datas[i-1]._g, imu_datas[i]._g, imu_datas[i-1]._t, imu_datas[i]._t, time);
                imu_data_temp._a = interEigenV(imu_datas[i-1]._a, imu_datas[i]._a, imu_datas[i-1]._t, imu_datas[i]._t, time);
                imu_data_temp._t = time;
                temp_imu_dat.push_back(imu_data_temp);
                break;
            }else{
                temp_imu_dat.push_back(imu_datas[i]);
                procceing_imu_id++;
            }
        }
        sycn_imu_datas_all.push_back(temp_imu_dat);
    }
    std::vector<std::vector<ORB_SLAM2::IMUData>> sycn_imu_datas=sycn_imu_datas_all;
    std::vector<cv::Mat> pose_vec_mat;
    
    for(int i=0; i<pose_vec.size(); i++){
        pose_vec_mat.push_back(ORB_SLAM2::Converter::toCvMat(pose_vec[i]));
    }
    
    Eigen::Vector3d bg=Eigen::Vector3d::Zero();
    Eigen::Vector3d ba=Eigen::Vector3d::Zero();
    std::vector<ORB_SLAM2::IMUPreintegrator> preints;
    updatePreInt(preints, sycn_imu_datas, ba, bg);
    Eigen::Vector3d new_bg = OptimizeInitialGyroBias(pose_vec_mat, preints, Tbc);
    bg=new_bg;
    preints.clear();
    updatePreInt(preints, sycn_imu_datas, ba, bg);
    double sstar;
    cv::Mat gwstar;
    CalGravityAndScale(pose_vec_mat, preints, ORB_SLAM2::Converter::toCvMat(Tbc), sstar, gwstar);
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = 1;
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    Eigen::Vector3d vhateig = ORB_SLAM2::Converter::toVector3d(vhat);
    Eigen::Matrix3d Rwi_ = Sophus::SO3::exp(vhateig*theta).matrix();
    
    Eigen::Matrix3d Rwi;
    Eigen::Vector3d bias_a;
    std::cout<<"s: "<<sstar<<std::endl;
    CalAccBias(pose_vec_mat, preints, sstar, gwstar, ORB_SLAM2::Converter::toCvMat(Tbc), Rwi, bias_a);
    std::cout<<"refined s: "<<sstar<<std::endl;
    std::cout<<"bias_a: "<<bias_a.transpose()<<std::endl;
    std::cout<<"bias_g: "<<new_bg.transpose()<<std::endl;
    std::cout<<"gravity: "<<gwstar.t()<<std::endl;
    Eigen::Vector3d g_b(0,0,9.8);
    std::cout<<"refined gravity: "<<(Rwi*g_b).transpose()<<std::endl;
    
    std::vector<ORB_SLAM2::NavState> states(pose_vec_mat.size());
    
    cv::Mat Tbc_mat=ORB_SLAM2::Converter::toCvMat(Tbc);
    cv::Mat Rbc = Tbc_mat.rowRange(0,3).colRange(0,3);
    cv::Mat pbc = Tbc_mat.rowRange(0,3).col(3);
    cv::Mat Rcb = Rbc.t();
    cv::Mat pcb = -Rcb*pbc;
    std::cout<<"pose_vec_mat: "<<pose_vec_mat.size()<<std::endl;
    std::cout<<"preints: "<<preints.size()<<std::endl;
    Eigen::Vector3d last_v;
    for(int i=0; i<pose_vec_mat.size(); i++){
        pose_vec_mat[i].col(3).rowRange(0,3)=pose_vec_mat[i].col(3).rowRange(0,3)*sstar;
    }
    for(int i=0; i<pose_vec_mat.size(); i++){
        ORB_SLAM2::NavState& ns=states[i];
        
        pose_vec_mat[i].col(3).rowRange(0,3)=pose_vec_mat[i].col(3).rowRange(0,3);
        cv::Mat wPc = pose_vec_mat[i].rowRange(0,3).col(3);                   // wPc
        cv::Mat Rwc = pose_vec_mat[i].rowRange(0,3).colRange(0,3);            // Rwc
        cv::Mat wPb = wPc + Rwc*pcb;
        ns.Set_Pos(ORB_SLAM2::Converter::toVector3d(wPb));
        ns.Set_Rot(ORB_SLAM2::Converter::toMatrix3d(Rwc*Rcb));
        ns.Set_BiasGyr(bg);
        ns.Set_BiasAcc(ba);
        ns.Set_DeltaBiasGyr(Eigen::Vector3d::Zero());
        ns.Set_DeltaBiasAcc(Eigen::Vector3d::Zero());
        Eigen::Vector3d veleig;
        if(i==pose_vec_mat.size()-1){
            ns.Set_Vel(last_v);
        }else{
            double dt = preints[i+1].getDeltaTime();  
            cv::Mat dp = ORB_SLAM2::Converter::toCvMat(preints[i+1].getDeltaP());       // deltaP
            cv::Mat Jpba = ORB_SLAM2::Converter::toCvMat(preints[i+1].getJPBiasa());    // J_deltaP_biasa
            cv::Mat wPcnext = pose_vec_mat[i+1].rowRange(0,3).col(3);           // wPc next
            cv::Mat Rwcnext = pose_vec_mat[i+1].rowRange(0,3).colRange(0,3);    // Rwc next
            cv::Mat vel = - 1./dt*(wPc - wPcnext);
            //cv::Mat vel = - 1./dt*( (wPc - wPcnext) + (Rwc - Rwcnext)*pcb + Rwc*Rcb*(dp + Jpba*ORB_SLAM2::Converter::toCvMat(ba)) + 0.5*gwstar*dt*dt );
            //std::cout<<(wPc - wPcnext).t()<<std::endl;
            veleig = ORB_SLAM2::Converter::toVector3d(vel);
            ns.Set_Vel(veleig);
        }
        
        last_v=veleig;
    }
    
    for(int i=0; i<mp_posis.size(); i++){
        mp_posis[i]=mp_posis[i]*sstar;
    }
    show_pose_as_marker(states, Rwi_, "before_opti");
    show_mp_as_cloud(mp_posis, Rwi_, "chamo_target");
    show_mp_as_cloud(lidar_posis, Eigen::Matrix3d::Identity(), "/chamo/gps");

    GlobalBundleAdjustmentNavStatePRV(preints, states, Tbc, mp_infos, fx, fy, cx, cy, mp_posis, gwstar, 100);
    
    show_pose_as_marker(states, Rwi_, "before_opti");
    
    for (int i=0; i<states.size(); i=i+10){
        std::cout<<states[i].Get_BiasGyr().transpose()<<std::endl;
    }
    for (int i=0; i<states.size(); i=i+10){
        std::cout<<states[i].Get_BiasAcc().transpose()<<std::endl;
    }
    
    ros::spin();
        
}