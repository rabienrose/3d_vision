#include <string>
#include <fstream>
#include <memory>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "test_imu_tool/visual_tool.h"

void show_pose_as_marker(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& pose_vec, std::string topic){
    visualization::PoseVector poses_vis;
    for(int i=0; i<pose_vec.size(); i=i+1){
        visualization::Pose pose;
        pose.G_p_B = pose_vec[i].block(0,3,3,1);
        Eigen::Matrix3d rot_m = pose_vec[i].block(0,0,3,3);
        Eigen::Quaterniond rot_q(rot_m);
        pose.G_q_B = rot_q;

        pose.id =poses_vis.size();
        pose.scale = 0.2;
        pose.line_width = 0.02;
        pose.alpha = 1;
        poses_vis.push_back(pose);
    }
    visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", topic);
}

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, Eigen::Matrix3d Rwi_, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=Rwi_.transpose()*mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
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