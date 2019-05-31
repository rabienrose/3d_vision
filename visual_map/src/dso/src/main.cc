

#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

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

void findIDByName(std::vector<std::string>& names, int& re_id, std::string query_name){
    re_id=-1;
    for(int i=0; i<names.size(); i++){
        if(names[i]==query_name){
            re_id=i;
            return;
        }
    }
    return;
};
            
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    std::string bag_addr=argv[2];
    std::string img_topic=argv[3];

    std::vector<Eigen::Vector3d> re_traj;
    
    std::vector<Eigen::Matrix4d> poses_alin;
    std::vector<std::string> frame_names;
    std::string traj_file_addr = res_root+"/frame_pose_opt.txt";
    CHAMO::read_traj_file(traj_file_addr, poses_alin, frame_names);
    std::cout<<"frame_pose_opt: "<<poses_alin.size()<<std::endl;
    
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(img_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    rosbag::View::iterator it= view.begin();
    std::vector<Eigen::Vector3d> align_frame_posi2;
    std::vector<Eigen::Vector3d> frame_posi2;
    std::vector<int> frame_to_matched_id2;
    
    ImageFolderReader* reader = new ImageFolderReader("",res_root+"camera.txt", "", "");
    reader->setGlobalCalibration();
    dso::FullSystem* fullSystem = new dso::FullSystem();
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = false;
    
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            img_count++;
            if(img_count<0){
                continue;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                cv::Mat img= cv_ptr->image;
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                std::stringstream ss_time;
                ss_time<<"img_"<<img_count<<".jpg";
                int re_id;
                findIDByName(frame_names, re_id, ss_time.str());
                if(re_id!=-1){
                    Eigen::Matrix4d cur_pose=poses_alin[re_id];
                    ImageAndExposure* result = new ImageAndExposure(img.cols, img.rows, simg->header.stamp.toSec());
//                     result->image=img.data();
//                     fullSystem->addActiveFrame(result, img_count);
                }
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }

    return 0;
}