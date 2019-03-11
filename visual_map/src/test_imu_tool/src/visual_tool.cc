#include <string>
#include <fstream>
#include <memory>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "test_imu_tool/visual_tool.h"

void show_pose_as_marker(std::vector<ORB_SLAM2::NavState>& states, Eigen::Matrix3d Rwi_, std::string topic){
    visualization::PoseVector poses_vis;
    for(int i=0; i<states.size(); i=i+1){
        visualization::Pose pose;
        pose.G_p_B = Rwi_.transpose()*states[i].Get_P();
        Eigen::Quaterniond rot_q(Rwi_.transpose()*states[i].Get_RotMatrix());
        pose.G_q_B = rot_q;

        pose.id =poses_vis.size();
        pose.scale = 0.2;
        pose.line_width = 0.02;
        pose.alpha = 1;
        poses_vis.push_back(pose);
    }
    visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", topic);
}

void show_mp_as_cloud(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& mp_posis, Eigen::Matrix3d Rwi_, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=Rwi_.transpose()*mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}