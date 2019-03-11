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
#include "visualization/common-rviz-visualization.h"

std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

Eigen::Matrix3d getRotFromVector(Eigen::Vector3d a, Eigen::Vector3d b){
     Eigen::Vector3d v = a.cross(b);
     Eigen::Matrix3d v_hat;
     v_hat <<   0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
     double c= a.dot(b);
     Eigen::Matrix3d re=Eigen::Matrix3d::Identity()+v_hat+v_hat*v_hat/(1+c);
     return re;
}

double interDouble(double v1, double v2, double t1, double t2, double t3)
{
    return v1 + (v2 - v1) * (t3 - t1) / (t2 - t1);
}

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
    std::string img_time_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/camera_1_image_time.txt";
    std::ifstream infile_img_time(img_time_addr.c_str());
    Eigen::Matrix4d Tbc= Eigen::Matrix4d::Identity();
    Tbc<<-0.99999518, -0.00310315,  0.00007742,  0.05310734,
           -0.00124823,  0.42483127,  0.90527169,  0.00929943,
           -0.00284208,  0.90526723, -0.4248331,  -0.01557671,
           0.0,          0.0,          0.0,         1.0;
   float fx=541.39699791; 
   float fy=540.86222539; 
   float cx=474.6630494; 
   float cy=306.8632145; 
   std::unordered_map<std::string, double> img_time_map;
    while (true)
    {
        std::getline(infile_img_time, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        img_time_map[splited[0]]=atof(splited[1].c_str());
    }
    
    std::map<double, int> pose_list;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> pose_vec;
    std::map<int, int> frame_ids;
    std::string pose_addr="/home/chamo/Documents/work/orb_mapping_loc/traj.txt";
    std::ifstream infile_pose(pose_addr.c_str());
    while (true)
    {
        std::getline(infile_pose, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        std::string file_name= splited[0];
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        int frame_id=atoi(splited[1].c_str());
        pose(0,0)=atof(splited[2].c_str());
        pose(0,1)=atof(splited[3].c_str());
        pose(0,2)=atof(splited[4].c_str());
        pose(0,3)=atof(splited[5].c_str());
        pose(1,0)=atof(splited[6].c_str());
        pose(1,1)=atof(splited[7].c_str());
        pose(1,2)=atof(splited[8].c_str());
        pose(1,3)=atof(splited[9].c_str());
        pose(2,0)=atof(splited[10].c_str());
        pose(2,1)=atof(splited[11].c_str());
        pose(2,2)=atof(splited[12].c_str());
        pose(2,3)=atof(splited[13].c_str());
        pose_vec.push_back(pose);
        frame_ids[frame_id]=pose_vec.size()-1;
        if(img_time_map.count(file_name)!=0){
            pose_list[img_time_map[file_name]]=pose_vec.size()-1;
        }else{
            std::cout<<"image not exist!!"<<std::endl;
            return 0;
        }        
    }

    std::string imu_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/imu.txt";
    std::ifstream infile_imu(imu_addr.c_str());
    
    std::vector<ORB_SLAM2::IMUData> imu_datas;
    int imu_count=0;
    Eigen::Vector3d gravity(0,0,-9.8);
    Eigen::Vector3d bg=Eigen::Vector3d::Zero();
    Eigen::Vector3d ba=Eigen::Vector3d::Zero();
    
    while (true)
    {
        std::getline(infile_imu, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        double timestamp=atof(splited[0].c_str());
        double gx=atof(splited[1].c_str());
        double gy=atof(splited[2].c_str());
        double gz=atof(splited[3].c_str());
        double ax=atof(splited[4].c_str());
        double ay=atof(splited[5].c_str());
        double az=atof(splited[6].c_str());
        ORB_SLAM2::IMUData imu_data(gx, gy, gz, ax, ay, az, timestamp);
        imu_datas.push_back(imu_data);
    }
    std::string posi_addr="/home/chamo/Documents/work/orb_mapping_loc/posi.txt";
    std::ifstream infile_posi(posi_addr.c_str());
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> mp_posis;
    while (true)
    {
        std::getline(infile_posi, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        Eigen::Vector3d mp_posi;
        mp_posi.x()=atof(splited[0].c_str());
        mp_posi.y()=atof(splited[1].c_str());
        mp_posi.z()=atof(splited[2].c_str());
        mp_posis.push_back(mp_posi);
    }
    std::string kp_addr="/home/chamo/Documents/work/orb_mapping_loc/kps.txt";
    std::ifstream infile_kp(kp_addr.c_str());
    std::vector<cv::Point2f> kp_uvs;
    std::vector<int> kp_frameids;
    std::vector<int> kp_octoves;
    while (true)
    {
        std::getline(infile_kp, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        cv::Point2f uv;
        uv.x=atof(splited[0].c_str());
        uv.y=atof(splited[1].c_str());
        int octove=atoi(splited[2].c_str());
        int frame_id=atoi(splited[3].c_str());
        kp_uvs.push_back(uv);
        kp_octoves.push_back(octove);
        kp_frameids.push_back(frame_id);
    }
    std::string track_addr="/home/chamo/Documents/work/orb_mapping_loc/track.txt";
    std::vector<std::vector<int>> tracks;
    std::ifstream infile_track(track_addr);
    while (true)
    {
        std::getline(infile_track, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        std::vector<int> track;
        for(auto str: splited){
            track.push_back(atoi(str.c_str()));
        }
        tracks.push_back(track);
    }
    
    std::vector<std::vector<ORB_SLAM2::MP_INFO>> mp_infos;
    for(int i=0; i<tracks.size(); i++){
        std::vector<ORB_SLAM2::MP_INFO> track_info;
        for(int j=0; j<tracks[i].size(); j++){
            int kp_id=tracks[i][j];
            ORB_SLAM2::MP_INFO info;
            info.u=kp_uvs[kp_id].x;
            info.v=kp_uvs[kp_id].y;
            info.octove=kp_octoves[kp_id];
            info.frame_id=frame_ids[kp_frameids[kp_id]];
            info.mp_id=i;
            track_info.push_back(info);
        }
        mp_infos.push_back(track_info);
    }
    
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lidar_posis;
    std::vector<Eigen::Quaterniond> lidar_dirs;
    std::string lidar_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/wayz_2018_11_26_result/interpolation_traj_camera_right.txt";
    std::ifstream infile_lidar(lidar_addr);
    while (true)
    {
        std::getline(infile_lidar, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        Eigen::Vector3d posi;
        posi.x()=atof(splited[6].c_str());
        posi.y()=atof(splited[10].c_str());
        posi.z()=atof(splited[14].c_str());
        lidar_posis.push_back(posi);
    }
    
//     Eigen::Matrix<double, 3, 4> proj_m=Eigen::Matrix<double, 3, 4>::Zero();
//     proj_m(0,0)=fx;
//     proj_m(1,1)=fy;
//     proj_m(0,2)=cx;
//     proj_m(1,2)=cy;
//     proj_m(2,2)=1;
//     double tot_err=0;
//     int proj_err_count=0;
//     for(int i=0; i<mp_infos.size(); i++){
//         for (int j=0; j<mp_infos[i].size(); j++){
//             Eigen::Matrix4d pose = pose_vec[mp_infos[i][j].frame_id].inverse();
//             Eigen::Vector3d posi=mp_posis[mp_infos[i][j].mp_id];
//             Eigen::Vector4d pose_homo; 
//             pose_homo(3,1)=1;
//             pose_homo.block<3,1>(0,0)=posi;
//             Eigen::Vector3d projected_vec= proj_m*pose*pose_homo;
//             double u=projected_vec(0)/projected_vec(2);
//             double v=projected_vec(1)/projected_vec(2);
//             double dist=sqrt((u-mp_infos[i][j].u)*(u-mp_infos[i][j].u)+(v-mp_infos[i][j].v)*(v-mp_infos[i][j].v));
//             tot_err=tot_err+dist;
//             proj_err_count++;
//         }
//     }
    //std::cout<<"tot proj err: "<<tot_err/proj_err_count<<std::endl;
    
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
        //sycn_imu_datas.push_back(sycn_imu_datas_all[i]);
    }
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
    //std::cout<<ORB_SLAM2::IMUData::getGyrBiasRW2()<<std::endl;       // Gyroscope bias random walk, covariance INVERSE
    //std::cout<<ORB_SLAM2::IMUData::getAccBiasRW2()<<std::endl;
//     for (int i=0; i<states.size(); i=i+10){
//         std::cout<<states[i].Get_V().norm()<<std::endl;
//     }
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

//     Eigen::Vector3d g_c=Eigen::Vector3d::Zero();
//     Eigen::Vector3d a_c=Eigen::Vector3d::Zero();
//     for(int i=100; i<1100;i++){
//         g_c = g_c+sycn_imu_datas[0][i]._g;
//         a_c = a_c+sycn_imu_datas[0][i]._a;
//     }
//     Eigen::Vector3d g_av=g_c/1000;
//     Eigen::Vector3d a_av=a_c/1000;
//     std::cout<<g_av.transpose()<<std::endl;
//     
//     std::cout<<"gravity norm: "<<cv::norm(gwstar)<<std::endl;
//     std::cout<<"imu ori: "<<a_av.transpose()<<std::endl;
//     bg=new_bg;
    //ba=bias_a;
    
//     Eigen::Vector3d a_b_norm = a_av.normalized();
//     Eigen::Vector3d a_w(0,0,1);
//     Eigen::Matrix3d Rib = getRotFromVector(a_b_norm, a_w);
//     
//     ORB_SLAM2::NavState ns;
//     ns.Set_Pos(Eigen::Vector3d::Zero());
//     ns.Set_Vel(Eigen::Vector3d::Zero());
//     ns.Set_BiasGyr(bg);
//     ns.Set_BiasAcc(ba);
//     Eigen::Matrix3d Rcb=Tbc.block(0,0,3,3).transpose();
//     ns.Set_Rot(Rcb);
//     //ns.Set_Rot(Rib);
//     for(int i=1; i<1000;i++){
//         const ORB_SLAM2::IMUData& cur_imu = imu_datas[i];
//         const ORB_SLAM2::IMUData& last_imu = imu_datas[i-1];
//         double dt = cur_imu._t - last_imu._t;
//         ORB_SLAM2::IMUPreintegrator imupreint;
//         imupreint.update(cur_imu._g - bg, cur_imu._a - ba, dt);
//         ORB_SLAM2::Converter::updateNS(ns, imupreint, Rwi*g_b);
//         //ORB_SLAM2::Converter::updateNS(ns, imupreint, gravity);
//     }
//     std::cout<<ns.Get_P().transpose()<<std::endl;
        
}