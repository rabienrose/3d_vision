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
        
        std::vector<std::vector<orb_slam::IMUData>> sycn_imu_datas;
        std::vector<cv::Mat> pose_vec_mat;
        for(int i=0; i<map.frames.size(); i++){
            pose_vec_mat.push_back(orb_slam::Converter::toCvMat(map.frames[i]->getPose()));
            std::vector<orb_slam::IMUData> imudatas;
            sycn_imu_datas.push_back(imudatas);
        }
        for(int i=1; i<map.frames.size(); i++){
        //for(int i=jj-wind_size; i<jj+wind_size; i++){
            CHECK_EQ(map.frames[i-1]->acces.size(), map.frames[i-1]->gyros.size());
            CHECK_EQ(map.frames[i-1]->acces.size(), map.frames[i-1]->imu_times.size());
            
            std::vector<orb_slam::IMUData> imudatas;
            if(map.frames[i]->acces.size()>0){
                for(int j=0; j<map.frames[i-1]->acces.size(); j++){
                    orb_slam::IMUData imu_data;
                    imu_data._a=map.frames[i-1]->acces[j];
                    imu_data._g=map.frames[i-1]->gyros[j];
                    imu_data._t=map.frames[i-1]->imu_times[j];
                    sycn_imu_datas[i].push_back(imu_data);
                }       
                orb_slam::IMUData imu_data;
                imu_data._a=map.frames[i]->acces[0];
                imu_data._g=map.frames[i]->gyros[0];
                imu_data._t=map.frames[i]->imu_times[0];
                sycn_imu_datas[i].push_back(imu_data);
            }
        }
        
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
                
//                 Eigen::Matrix<double,3, 4> k_mat=Eigen::Matrix<double,3, 4>::Zero();
//                 k_mat(0,0)=fx;
//                 k_mat(1,1)=fy;
//                 k_mat(0,2)=cx;
//                 k_mat(1,2)=cy;
//                 k_mat(2,2)=1;
//                 std::cout<<"==================="<<std::endl;
//                 std::cout<<info.frame_id<<std::endl;
//                 std::cout<<map.mappoints[i]->track[j].frame->id<<std::endl;
//                 std::cout<<pose_vec_mat[info.frame_id]<<std::endl;
//                 std::cout<<map.mappoints[i]->track[j].frame->getPose()<<std::endl;
//                 Eigen::Matrix<double,3, 4> proj_mat=k_mat*orb_slam::Converter::toMatrix4d(pose_vec_mat[info.frame_id]).inverse();
//                 Eigen::Vector4d posi_homo;
//                 posi_homo.block(0,0,3,1)=map.mappoints[i]->position;
//                 posi_homo(3)=1;
//                 Eigen::Vector3d proj_homo = proj_mat*posi_homo;
//                 //std::cout<<proj_mat<<std::endl;
//                 double u=proj_homo(0)/proj_homo(2);
//                 double v=proj_homo(1)/proj_homo(2);
//                 //cv::Point2f uv= map.frames[i]->kps[j].pt;
//                 //std::cout<<u<<":"<<v<<"     "<<uv.x<<":"<<uv.y<<std::endl;
//                 
//                 float proj_err=sqrt((u-info.u)*(u-info.u)+(v-info.v)*(v-info.v));
                
                //std::cout<<proj_err<<std::endl;
                track_info.push_back(info);
            }
            mp_posis.push_back(map.mappoints[i]->position);
            mp_infos.push_back(track_info);
        }
        
//         std::ofstream f;
//         f.open(res_root+"/chamo.txt");
        int wind_size=5;
//         for(int jj=wind_size+1; jj<map.frames.size()-wind_size-1; jj=jj+1){
            //std::cout<<map.frames[jj]->frame_file_name<<std::endl;
            
            
            Eigen::Vector3d bg=Eigen::Vector3d::Zero();
            Eigen::Vector3d ba=Eigen::Vector3d::Zero();
            std::vector<orb_slam::IMUPreintegrator> preints;
            updatePreInt(preints, sycn_imu_datas, ba, bg);
//             for(int i=0; i<preints.size(); i++){
//                 if(i<jj-wind_size||i>jj+wind_size){
//                     preints[i]._delta_time=0;
//                 }
//             }
            
            Eigen::Vector3d new_bg = OptimizeInitialGyroBias(pose_vec_mat, preints, Tbc);
            bg=new_bg;
            
            preints.clear();
            updatePreInt(preints, sycn_imu_datas, ba, bg);
//             for(int i=0; i<preints.size(); i++){
//                 if(i<jj-wind_size||i>jj+wind_size){
//                     preints[i]._delta_time=0;
//                 }
//             }

            double sstar;
            cv::Mat gwstar;
            double scale_confi;
            double grav_confi;
            CalGravityAndScale(pose_vec_mat, preints, orb_slam::Converter::toCvMat(Tbc), sstar, gwstar, scale_confi, grav_confi);
//             f<<map.frames[jj]->frame_file_name<<",";
//             f<<std::setprecision(15)<<map.frames[jj]->time_stamp<<",";
//             if(map.frames[jj]->acces.size()>0){
//                 f<<map.frames[jj]->acces[0].norm()<<",";
//                 f<<map.frames[jj]->gyros[0].norm()<<",";
//             }else{
//                 f<<9999<<",";
//                 f<<9999<<",";
//             }
//             f<<scale_confi<<",";
//             f<<grav_confi<<",";
//             f<<sstar<<",";
//             f<<std::endl;
            cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
            gI.at<float>(2) = 1;
            cv::Mat gwn = gwstar/cv::norm(gwstar);
            cv::Mat gIxgwn = gI.cross(gwn);
            double normgIxgwn = cv::norm(gIxgwn);
            cv::Mat vhat = gIxgwn/normgIxgwn;
            double theta = std::atan2(normgIxgwn,gI.dot(gwn));
            Eigen::Vector3d vhateig = orb_slam::Converter::toVector3d(vhat);
            Eigen::Matrix3d Rwi_ = Sophus::SO3::exp(vhateig*theta).matrix();
            
            Eigen::Matrix3d Rwi=Rwi_;
        
            Eigen::Vector3d bias_a=Eigen::Vector3d::Zero();
            //std::cout<<"s: "<<sstar<<std::endl;

            Eigen::Vector3d g_b(0,0,9.8);
            cv::Mat g_b_m=orb_slam::Converter::toCvMat(g_b);
            CalAccBias(pose_vec_mat, preints, sstar, gwstar, orb_slam::Converter::toCvMat(Tbc), Rwi, bias_a);
            std::cout<<"preints: "<<preints.size()<<std::endl;
            std::cout<<"sycn_imu_datas:"<<sycn_imu_datas.size()<<std::endl;
            preints.clear();
            updatePreInt(preints, sycn_imu_datas, bias_a, bg);
            std::cout<<"preints: "<<preints.size()<<std::endl;
            std::cout<<"refined s: "<<sstar<<std::endl;
            std::cout<<"bias_a: "<<bias_a.transpose()<<std::endl;
            std::cout<<"bias_g: "<<new_bg.transpose()<<std::endl;
            std::cout<<"gravity: "<<gwstar.t()<<std::endl;
            
            std::cout<<"refined gravity: "<<(Rwi*g_b).transpose()<<std::endl;
            std::vector<orb_slam::NavState> states(pose_vec_mat.size());
            
            cv::Mat Tbc_mat=orb_slam::Converter::toCvMat(Tbc);
            cv::Mat Rwi_mat=orb_slam::Converter::toCvMat(Rwi);
            Eigen::Matrix4d Twi = Eigen::Matrix4d::Identity();
            Twi.block(0,0,3,3)=Rwi;
            cv::Mat Twi_mat=orb_slam::Converter::toCvMat(Twi);
            cv::Mat Rbc = Tbc_mat.rowRange(0,3).colRange(0,3);
            cv::Mat pbc = Tbc_mat.rowRange(0,3).col(3);
            cv::Mat Rcb = Rbc.t();
            cv::Mat pcb = -Rcb*pbc;
            std::cout<<"states: "<<states.size()<<std::endl;
            std::cout<<"frames: "<<map.frames.size()<<std::endl;
            Eigen::Vector3d last_v;
            for(int i=0; i<pose_vec_mat.size(); i++){
                pose_vec_mat[i].col(3).rowRange(0,3)=pose_vec_mat[i].col(3).rowRange(0,3)*sstar;
                pose_vec_mat[i]=Twi_mat.t()*pose_vec_mat[i];
            }
            for(int i=0; i<pose_vec_mat.size(); i++){
                orb_slam::NavState& ns=states[i];
                cv::Mat wPc = pose_vec_mat[i].rowRange(0,3).col(3);                   // wPc
                cv::Mat Rwc = pose_vec_mat[i].rowRange(0,3).colRange(0,3);            // Rwc
                cv::Mat wPb = wPc + Rwc*pcb;
                ns.Set_Pos(orb_slam::Converter::toVector3d(wPb));
                ns.Set_Rot(orb_slam::Converter::toMatrix3d(Rwc*Rcb));
                //std::cout<<"Rwc*Rcb: "<<Rwc*Rcb<<std::endl;
                ns.Set_BiasGyr(bg);
                ns.Set_BiasAcc(bias_a);
                ns.Set_DeltaBiasGyr(Eigen::Vector3d::Zero());
                ns.Set_DeltaBiasAcc(Eigen::Vector3d::Zero());
                Eigen::Vector3d veleig;
                if(i==pose_vec_mat.size()-1){
                    ns.Set_Vel(last_v);
                }else{
                    double dt = preints[i+1].getDeltaTime();  
                    if(dt==0){
                        ns.Set_Vel(last_v);
                    }else{
                        //cv::Mat dp = orb_slam::Converter::toCvMat(preints[i+1].getDeltaP());       // deltaP
                        //cv::Mat Jpba = orb_slam::Converter::toCvMat(preints[i+1].getJPBiasa());    // J_deltaP_biasa
                        cv::Mat wPcnext = pose_vec_mat[i+1].rowRange(0,3).col(3);           // wPc next
                        //cv::Mat Rwcnext = pose_vec_mat[i+1].rowRange(0,3).colRange(0,3);    // Rwc next
                        cv::Mat vel = - 1./dt*(wPc - wPcnext);
                        //cv::Mat vel = - 1./dt*( (wPc - wPcnext) + (Rwc - Rwcnext)*pcb + Rwc*Rcb*(dp + Jpba*orb_slam::Converter::toCvMat(ba)) + 0.5*orb_slam::Converter::toCvMat(g_b)*dt*dt );
                        //std::cout<<(wPc - wPcnext).t()<<std::endl;
                        veleig = orb_slam::Converter::toVector3d(vel);
                        ns.Set_Vel(veleig);
                    }
                }
                last_v=veleig;
            }
            
            for(int i=0; i<mp_posis.size(); i++){
                mp_posis[i]=Rwi.transpose()*mp_posis[i]*sstar;
                //mp_posis[i]=mp_posis[i]*sstar;
            }
            CHECK_EQ(preints.size(), states.size());
            GlobalBundleAdjustmentNavStatePRV(preints, states, Tbc, mp_infos, fx, fy, cx, cy, mp_posis, orb_slam::Converter::toCvMat(g_b), 100);
//         }
            CHECK_EQ(map.mappoints.size(), mp_posis.size());
            for(int i=0; i<map.mappoints.size(); i++){
                map.mappoints[i]->position=mp_posis[i];
            }

            for(int i=0; i<states.size(); i++){
                Eigen::Matrix4d T_w_b=Eigen::Matrix4d::Identity(); 
                T_w_b.block(0,0,3,3)=states[i].Get_RotMatrix();
                T_w_b.block(0,3,3,1)=states[i].Get_P();
                Eigen::Matrix4d T_w_i=Eigen::Matrix4d::Identity(); 
                T_w_i.block(0,0,3,3)=Rwi;
                Eigen::Matrix4d T_i_c = T_w_b*Tbc;
                map.frames[i]->position=T_i_c.block(0,3,3,1);
                Eigen::Matrix3d tempm=T_i_c.block(0,0,3,3);
                map.frames[i]->direction=Eigen::Quaterniond(tempm);
            }
            map.UpdatePoseEdge();
            vm::save_visual_map(map, res_root+"/imu_"+map_name);
            
        return;
    }
}
