#include <string>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <math.h>
#include "imu_tools.h"
#include "NavState.h"
#include "IMUConverter.h"
#include "read_write_data_lib/read_write.h"
#include "optimizer_tool/optimizer_tool.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"

namespace OptimizerTool{
    Eigen::Vector3d interEigenV(Eigen::Vector3d v1, Eigen::Vector3d v2, double t1, double t2, double t3){
        return v1 + (v2 - v1) * (t3 - t1) / (t2 - t1);
    }

    void updatePreInt(std::vector<orb_slam::IMUPreintegrator>& preints, std::vector<std::vector<orb_slam::IMUData>>& sycn_imu_datas,
        Eigen::Vector3d ba, Eigen::Vector3d bg
    ){
        for(int i=0; i<sycn_imu_datas.size(); i++){
            orb_slam::IMUPreintegrator temp_preint;
            temp_preint.reset();
            for(int j=1; j<sycn_imu_datas[i].size(); j++){
                double dt=sycn_imu_datas[i][j]._t-sycn_imu_datas[i][j-1]._t;
                temp_preint.update(sycn_imu_datas[i][j]._g - bg, sycn_imu_datas[i][j]._a - ba, dt);
            }
            preints.push_back(temp_preint);
        }
    }

    void optimize_imu(std::string res_root, std::string map_name) {
        vm::VisualMap map;
        vm::loader_visual_map(map, res_root+"/"+map_name);
        map.ComputeUniqueId();
        CHECK_GT(map.frames.size(), 0);
        
        Eigen::Matrix3d temp_rot(map.Tbc_qua);
        Eigen::Matrix4d Tbc= Eigen::Matrix4d::Identity();
        Tbc.block(0,0,3,3)=temp_rot;
        Tbc.block(0,3,3,1)=map.Tbc_posi;

        float fx=map.frames[0]->fx; 
        float fy=map.frames[0]->fy; 
        float cx=map.frames[0]->cx;  
        float cy=map.frames[0]->cy;
        std::vector<Eigen::Vector3d> mp_posis;
        std::vector<std::vector<orb_slam::MP_INFO>> mp_infos;
        for(int i=0; i<map.mappoints.size(); i++){
            std::vector<orb_slam::MP_INFO> track_info;
            for(int j=0; j<map.mappoints[i]->track.size(); j++){
                int kp_id=map.mappoints[i]->track[j].kp_ind;
                orb_slam::MP_INFO info;
                info.u=map.mappoints[i]->track[j].frame->kps[kp_id].pt.x;
                info.v=map.mappoints[i]->track[j].frame->kps[kp_id].pt.y;
                info.octove=map.mappoints[i]->track[j].frame->kps[kp_id].octave;
                info.frame_id=map.mappoints[i]->track[j].frame->id;
                info.mp_id=i;
                track_info.push_back(info);
            }
            mp_posis.push_back(map.mappoints[i]->position);
            mp_infos.push_back(track_info);
        }
        int wind_size=20;
        for(int jj=wind_size+1; jj<map.frames.size()-wind_size-1; jj++){
            std::vector<std::vector<orb_slam::IMUData>> sycn_imu_datas;
            std::vector<cv::Mat> pose_vec_mat;
            for(int i=jj-wind_size; i<jj+wind_size; i++){
                CHECK_EQ(map.frames[i-1]->acces.size(), map.frames[i-1]->gyros.size());
                CHECK_EQ(map.frames[i-1]->acces.size(), map.frames[i-1]->imu_times.size());
                
                std::vector<orb_slam::IMUData> imudatas;
                if(map.frames[i]->acces.size()>0){
                    for(int j=0; j<map.frames[i-1]->acces.size(); j++){
                        orb_slam::IMUData imu_data;
                        imu_data._a=map.frames[i-1]->acces[j];
                        imu_data._g=map.frames[i-1]->gyros[j];
                        imu_data._t=map.frames[i-1]->imu_times[j];
                        imudatas.push_back(imu_data);
                    }       
                    orb_slam::IMUData imu_data;
                    imu_data._a=map.frames[i]->acces[0];
                    imu_data._g=map.frames[i]->gyros[0];
                    imu_data._t=map.frames[i]->imu_times[0];
                    imudatas.push_back(imu_data);
                }
                if(imudatas.size()>0){
                    sycn_imu_datas.push_back(imudatas);
                    pose_vec_mat.push_back(orb_slam::Converter::toCvMat(map.frames[i]->getPose()));
                }
            }
            
            CHECK_EQ(sycn_imu_datas.size(), pose_vec_mat.size());
            
            Eigen::Vector3d bg=Eigen::Vector3d::Zero();
            Eigen::Vector3d ba=Eigen::Vector3d::Zero();
            std::vector<orb_slam::IMUPreintegrator> preints;
            updatePreInt(preints, sycn_imu_datas, ba, bg);
            
            Eigen::Vector3d new_bg = OptimizeInitialGyroBias(pose_vec_mat, preints, Tbc);
            bg=new_bg;
            
            preints.clear();
            updatePreInt(preints, sycn_imu_datas, ba, bg);

            double sstar;
            cv::Mat gwstar;
            CalGravityAndScale(pose_vec_mat, preints, orb_slam::Converter::toCvMat(Tbc), sstar, gwstar);
            
//             cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
//             gI.at<float>(2) = 1;
//             cv::Mat gwn = gwstar/cv::norm(gwstar);
//             cv::Mat gIxgwn = gI.cross(gwn);
//             double normgIxgwn = cv::norm(gIxgwn);
//             cv::Mat vhat = gIxgwn/normgIxgwn;
//             double theta = std::atan2(normgIxgwn,gI.dot(gwn));
//             Eigen::Vector3d vhateig = orb_slam::Converter::toVector3d(vhat);
//             Eigen::Matrix3d Rwi_ = Sophus::SO3::exp(vhateig*theta).matrix();
//             
//             Eigen::Matrix3d Rwi;
//             Eigen::Vector3d bias_a;
//             std::cout<<"s: "<<sstar<<std::endl;
//             CalAccBias(pose_vec_mat, preints, sstar, gwstar, orb_slam::Converter::toCvMat(Tbc), Rwi, bias_a);
//             std::cout<<"preints: "<<preints.size()<<std::endl;
//             std::cout<<"sycn_imu_datas:"<<sycn_imu_datas.size()<<std::endl;
//             preints.clear();
//             updatePreInt(preints, sycn_imu_datas, bias_a, bg);
//             std::cout<<"preints: "<<preints.size()<<std::endl;
//             std::cout<<"refined s: "<<sstar<<std::endl;
//             std::cout<<"bias_a: "<<bias_a.transpose()<<std::endl;
//             std::cout<<"bias_g: "<<new_bg.transpose()<<std::endl;
//             std::cout<<"gravity: "<<gwstar.t()<<std::endl;
//             Eigen::Vector3d g_b(0,0,9.8);
//             std::cout<<"refined gravity: "<<(Rwi*g_b).transpose()<<std::endl;
            std::cout<<sstar<<std::endl;
        }
        
        return;
        

//         
//         std::vector<orb_slam::NavState> states(pose_vec_mat.size());
//         
//         cv::Mat Tbc_mat=orb_slam::Converter::toCvMat(Tbc);
//         cv::Mat Rbc = Tbc_mat.rowRange(0,3).colRange(0,3);
//         cv::Mat pbc = Tbc_mat.rowRange(0,3).col(3);
//         cv::Mat Rcb = Rbc.t();
//         cv::Mat pcb = -Rcb*pbc;
//         std::cout<<"states: "<<states.size()<<std::endl;
//         
//         Eigen::Vector3d last_v;
//         for(int i=0; i<pose_vec_mat.size(); i++){
//             pose_vec_mat[i].col(3).rowRange(0,3)=pose_vec_mat[i].col(3).rowRange(0,3)*sstar;
//         }
//         for(int i=0; i<pose_vec_mat.size(); i++){
//             orb_slam::NavState& ns=states[i];
//             
//             pose_vec_mat[i].col(3).rowRange(0,3)=pose_vec_mat[i].col(3).rowRange(0,3);
//             cv::Mat wPc = pose_vec_mat[i].rowRange(0,3).col(3);                   // wPc
//             cv::Mat Rwc = pose_vec_mat[i].rowRange(0,3).colRange(0,3);            // Rwc
//             cv::Mat wPb = wPc + Rwc*pcb;
//             ns.Set_Pos(orb_slam::Converter::toVector3d(wPb));
//             ns.Set_Rot(orb_slam::Converter::toMatrix3d(Rwc*Rcb));
//             //std::cout<<"Rwc*Rcb: "<<Rwc*Rcb<<std::endl;
//             ns.Set_BiasGyr(bg);
//             ns.Set_BiasAcc(bias_a);
//             ns.Set_DeltaBiasGyr(Eigen::Vector3d::Zero());
//             ns.Set_DeltaBiasAcc(Eigen::Vector3d::Zero());
//             Eigen::Vector3d veleig;
//             if(i==pose_vec_mat.size()-1){
//                 ns.Set_Vel(last_v);
//             }else{
//                 double dt = preints[i+1].getDeltaTime();  
//                 cv::Mat dp = orb_slam::Converter::toCvMat(preints[i+1].getDeltaP());       // deltaP
//                 cv::Mat Jpba = orb_slam::Converter::toCvMat(preints[i+1].getJPBiasa());    // J_deltaP_biasa
//                 cv::Mat wPcnext = pose_vec_mat[i+1].rowRange(0,3).col(3);           // wPc next
//                 cv::Mat Rwcnext = pose_vec_mat[i+1].rowRange(0,3).colRange(0,3);    // Rwc next
//                 cv::Mat vel = - 1./dt*(wPc - wPcnext);
//                 //cv::Mat vel = - 1./dt*( (wPc - wPcnext) + (Rwc - Rwcnext)*pcb + Rwc*Rcb*(dp + Jpba*orb_slam::Converter::toCvMat(ba)) + 0.5*gwstar*dt*dt );
//                 //std::cout<<(wPc - wPcnext).t()<<std::endl;
//                 veleig = orb_slam::Converter::toVector3d(vel);
//                 ns.Set_Vel(veleig);
//             }
//             
//             last_v=veleig;
//         }
//         
//         for(int i=0; i<mp_posis.size(); i++){
//             mp_posis[i]=mp_posis[i]*sstar;
//         }
// 
//         GlobalBundleAdjustmentNavStatePRV(preints, states, Tbc, mp_infos, fx, fy, cx, cy, mp_posis, gwstar, 100);

    }
}
