

#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/ImmaturePoint.h"
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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>

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

void addMP(PointHessian* ph, std::vector<Eigen::Vector3d>& mp_list, Eigen::Matrix3f& K, float max_range){
    float fxi = 1/K(0,0);
    float fyi = 1/K(1,1);
    float cxi = -K(0,2) / K(0,0);
    float cyi = -K(1,2) / K(1,1);
    float depth = 1.0f / ph->idepth;
    
    //std::cout<<ph->idepth_scaled<<std::endl;
    //for(int pnt=0;pnt<patternNum;pnt++){
        //int dx = patternP[pnt][0];
        //int dy = patternP[pnt][1];
        int dx = 0;
        int dy = 0;
        Eigen::Vector3d mp_posi;
        mp_posi(0) = ((ph->u+dx)*fxi + cxi) * depth;
        mp_posi(1) = ((ph->v+dy)*fyi + cyi) * depth;
        mp_posi(2) = depth/**(1 + 2*fxi * (rand()/(float)RAND_MAX-0.5f))*/;
        if(mp_posi(2)>max_range){
            return;
        }
        FrameHessian* host = ph->host;
        Eigen::Vector4d mp_posi_homo;
        mp_posi_homo.block(0,0,3,1)=mp_posi;
        mp_posi_homo(3)=1;
        mp_posi_homo = host->shell->camToWorld.matrix()*mp_posi_homo;
        mp_posi=mp_posi_homo.block(0,0,3,1);
        if(mp_posi(2)<-1){
            //return;
        }
        for(int i=0;i<mp_list.size() ;i++){
            if((mp_list[i]-mp_posi).norm()<0.01){
                return;
            }
        }
        mp_list.push_back(mp_posi);
    //}
}

void addMP(ImmaturePoint* ph, std::vector<Eigen::Vector3d>& mp_list, Eigen::Matrix3f& K){
    float fxi = 1/K(0,0);
    float fyi = 1/K(1,1);
    float cxi = -K(0,2) / K(0,0);
    float cyi = -K(1,2) / K(1,1);
    float depth = 1.0f / ((ph->idepth_max+ph->idepth_min)*0.5f);
    //std::cout<<ph->idepth_scaled<<std::endl;
    int dx = 0;
    int dy = 0;
    Eigen::Vector3d mp_posi;
    mp_posi(0) = ((ph->u+dx)*fxi + cxi) * depth;
    mp_posi(1) = ((ph->v+dy)*fyi + cyi) * depth;
    mp_posi(2) = depth;
    FrameHessian* host = ph->host;
    Eigen::Vector4d mp_posi_homo;
    mp_posi_homo.block(0,0,3,1)=mp_posi;
    mp_posi_homo(3)=1;
    mp_posi_homo = host->shell->camToWorld.matrix()*mp_posi_homo;
    mp_posi=mp_posi_homo.block(0,0,3,1);
    
    mp_list.push_back(mp_posi);
}

void saveToPcd(std::string semi_pcd, std::vector<Eigen::Vector3d>& mp_display){
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width    = mp_display.size();
    cloud->height   = 1; 
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = mp_display[i](0);
        cloud->points[i].y = mp_display[i](1);
        cloud->points[i].z = mp_display[i](2);
    }

    pcl::io::savePCDFileBinary(semi_pcd, *cloud);
}
            
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    std::string bag_addr=argv[2];
    std::string img_topic=argv[3];
    float max_Range=atof(argv[4]);
    int dense_lv=atof(argv[5]);
    int output_step=atof(argv[6]);

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
    int output_count=0;
    rosbag::View::iterator it= view.begin();
    std::vector<Eigen::Vector3d> align_frame_posi2;
    std::vector<Eigen::Vector3d> frame_posi2;
    std::vector<int> frame_to_matched_id2;
    
    ImageFolderReader* reader = new ImageFolderReader("",res_root+"camera.txt", "", "");
    reader->setGlobalCalibration();
    dso::FullSystem* fullSystem = new dso::FullSystem();
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = false;
    
    Eigen::Matrix3f K;
    int w;
    int h;
    reader->getCalibMono(K, w, h);
    float fxi = 1/K(0,0);
    float fyi = 1/K(1,1);
    float cxi = -K(0,2) / K(0,0);
    float cyi = -K(1,2) / K(1,1);
    
    std::vector<Eigen::Vector3d> traj_display;
    std::vector<Eigen::Vector3d> mp_display;
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            img_count++;
            //if(img_count<0){
            if(img_count<100){
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
                    cv::Mat img_float;
                    img.convertTo(img_float, CV_32F, 1.0, 0);
                    result->image=(float*)img_float.data;
                    fullSystem->addActiveFrame(result, img_count, cur_pose);
                    int temp_step=10;
                    if(output_step!=-1){
                        temp_step=output_step;
                    }
                    if(img_count%temp_step==0){
                        output_count++;
                        if(output_step!=-1){
                            mp_display.clear();
                        }
                        for(FrameHessian* fh : fullSystem->frameHessians){
                            Eigen::Vector3d posi = fh->shell->camToWorld.matrix3x4().block(0,3,3,1);
                            traj_display.push_back(posi);
//                             for(PointHessian* ph : fh->pointHessiansOut){
//                                 addMP(ph, mp_display, K);
//                             }
                            if(dense_lv<=2){
                                for(PointHessian* ph : fh->pointHessiansMarginalized){
                                    addMP(ph, mp_display, K, max_Range);
                                }
                                if(dense_lv==2){
                                    for(PointHessian* ph : fh->pointHessians){
                                        addMP(ph, mp_display, K, max_Range);
                                    }
                                }
                            }
                            
                           
//                             for(ImmaturePoint* ph : fh->immaturePoints){
//                                 addMP(ph, mp_display, K);
//                             }
                        }
                        if(output_step!=-1){
                            if(mp_display.size()>1000){
                                std::stringstream ss;
                                ss<<res_root<<"/semi_pc_"<<output_count+100000<<".pcd";
                                saveToPcd(ss.str(), mp_display);
                            }
                        }
                    }
                    if(img_count%10==0){
                        show_mp_as_cloud(traj_display, "dso_traj");
                        show_mp_as_cloud(mp_display, "dso_mp");
                        std::cout<<"mp count: "<<mp_display.size()<<std::endl;
                    }
                }
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    
    if(output_step==-1){
        
        std::string semi_pcd=res_root+"/semi_pc.pcd";
        saveToPcd(semi_pcd, mp_display);
    }
    


    return 0;
}
