#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}
    
void show_pose_as_marker(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& rots, std::string topic){
    visualization::PoseVector poses_vis;
    for(int i=0; i<posis.size(); i=i+1){
        visualization::Pose pose;
        pose.G_p_B = posis[i];
        pose.G_q_B = rots[i];

        pose.id =poses_vis.size();
        pose.scale = 0.2;
        pose.line_width = 0.02;
        pose.alpha = 1;
        poses_vis.push_back(pose);
    }
    visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", topic);
}
          
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];

    
    std::string posi_addr=res_root+"/mp_posi_opt.txt";
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(posi_addr, mp_posis);
    std::cout<<"mp_posis: "<<mp_posis.size()<<std::endl;
    show_mp_as_cloud(mp_posis, "mp_posi");
    
    std::vector<Eigen::Matrix4d> poses_alin;
    std::vector<std::string> frame_names;
    std::string traj_file_addr = res_root+"/frame_pose_opt.txt";
    CHAMO::read_traj_file(traj_file_addr, poses_alin, frame_names);
    std::cout<<"frame_pose_opt: "<<poses_alin.size()<<std::endl;
    std::vector<Eigen::Vector3d> traj_posi;
    for(int i=0; i<poses_alin.size(); i++){
        traj_posi.push_back(poses_alin[i].block(0,3,3,1));
    }
    show_mp_as_cloud(traj_posi, "traj_posi");
    
    ros::spin();

    return 0;
}