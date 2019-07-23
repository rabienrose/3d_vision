#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include <glog/logging.h>
#include <gflags/gflags.h>


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
    
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    
    std::vector<int> start_frame_ids;
    std::vector<int> end_frame_ids;
    
    start_frame_ids.push_back(0);
    start_frame_ids.push_back(300);
    end_frame_ids.push_back(299);
    end_frame_ids.push_back(500);

    vm::VisualMap map;
    vm::loader_visual_map(map, res_root);
    
    std::vector<std::vector<std::shared_ptr<vm::Frame>>> multi_trajs; 
    
    for(int j=0; j<100000; j++){
        for(int i=0; i<start_frame_ids.size(); i++){
            if(j==0){
                std::vector<std::shared_ptr<vm::Frame>> temp_empty;
                multi_trajs.push_back(temp_empty);
            }
            if(start_frame_ids[i]+j<=end_frame_ids[i]){
                CHECK_GT(map.frames.size(), start_frame_ids[i]+j);
                multi_trajs[i].push_back(map.frames[start_frame_ids[i]+j]);
            }
        }
    }
    
    std::cout<<"1111"<<std::endl;
    
    ros::Rate loop_rate(10);
    std::vector<Eigen::Vector3d> all_mps;
    for(int n=0; n<10000; n++){
        visualization::LineSegmentVector matches;
        for(int k=0; k<multi_trajs.size(); k++){
            if(n>=multi_trajs[k].size()){
                continue;
            }
            for(int j=0; j<multi_trajs[k][n]->obss.size(); j++){
                if(multi_trajs[k][n]->obss[j]!=nullptr){
                    visualization::LineSegment line_segment;
                    line_segment.from = multi_trajs[k][n]->position;
                    line_segment.scale = 0.03;
                    line_segment.alpha = 0.6;

                    line_segment.color.red = 255;
                    line_segment.color.green = 255;
                    line_segment.color.blue = 255;
                    line_segment.to = multi_trajs[k][n]->obss[j]->position;
                    all_mps.push_back(multi_trajs[k][n]->obss[j]->position);
                    matches.push_back(line_segment);
                }
            }
        }
        visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, "demo_matches");
        show_mp_as_cloud(all_mps, "demo_mp");
        if(ros::ok()){
            loop_rate.sleep();
        }else{
            break;
        }
    }
    ros::spin();
    return 0;
}