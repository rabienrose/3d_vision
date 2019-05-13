
#include <rovio/RovioFilter.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "read_write_data_lib/read_write.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "CoorConv.h"

typedef rovio::RovioFilter<rovio::FilterState<25, 4, 6, 1, 1>> FilterType; 
typedef typename FilterType::mtPrediction::mtMeas mtPredictionMeas;
typedef typename std::tuple_element<0, typename FilterType::mtUpdates>::type mtImgUpdate;
typedef typename std::tuple_element<1, typename FilterType::mtUpdates>::type mtPoseUpdate;
typedef typename mtImgUpdate::mtMeas mtImgMeas;
typedef typename mtPoseUpdate::mtMeas mtPoseMeas;

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

void convert_mat_float_eigen_double(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix, cv::Mat mat){
    matrix.resize(mat.rows, mat.cols);
    for(int i=0; i<mat.rows; i++){
        for(int j=0; j<mat.cols; j++){
            matrix(i, j) = mat.at<double>(i, j);
        }
    }
}

void convert_eigen_double_mat_float(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix, cv::Mat& mat){
    mat = cv::Mat(matrix.rows(), matrix.cols(), CV_32FC1);
    for(int i=0; i<matrix.rows(); i++){
        for(int j=0; j<matrix.cols(); j++){
            mat.at<float>(i, j)=matrix(i, j);
        }
    }
}

                                        
int main(int argc, char* argv[]){
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_addr=argv[1];
    std::string bag_addr_=argv[2];
    std::string img_topic=argv[3];
    std::string imu_topic=argv[4];
    std::string gps_topic=argv[5];
    std::shared_ptr<FilterType> mpFilter_;
    mtPredictionMeas predictionMeas_;
    mtImgUpdate *mpImgUpdate_;
    mtPoseUpdate *mpPoseUpdate_;
    mtImgMeas imgUpdateMeas_;
    mtPoseMeas poseUpdateMeas_;
    mpFilter_.reset(new FilterType);
    mpFilter_->readFromInfo(res_addr+"/rovio_default_config.info");
    mpFilter_->refreshProperties();
    std::string cam_addr=res_addr+"/camera_config.txt";
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    cv::Mat cam_inter_cv;
    cv::Mat cam_distort_cv;
    CHAMO::read_cam_info(cam_addr, cam_inter, cam_distort, Tbc);
    convert_eigen_double_mat_float(cam_inter, cam_inter_cv);
    convert_eigen_double_mat_float(cam_distort, cam_distort_cv);
    rovio::CameraCalibrationVector calibrations;
    rovio::CameraCalibration calibration;
    calibration.K_=cam_inter;
    calibration.distortionModel_=rovio::DistortionModel::RADTAN;
    calibration.distortionParams_=Eigen::VectorXd::Zero(5);
    calibration.hasIntrinsics_=true;
    calibrations.push_back(calibration);
    mpFilter_->setCameraCalibrations(calibrations);
    Eigen::Matrix4d Tcb  = Tbc.inverse();
    mpFilter_->setExtrinsics(Tcb.block(0,0,3,3), Tcb.block(0,3,3,1));
    std::string bag_addr=bag_addr_;
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(img_topic);
    topics.push_back(imu_topic);
    topics.push_back(gps_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    bool is_init=false;
    rosbag::View::iterator it= view.begin();
    Eigen::Vector3d anchorGps;
    int gps_count=0;
    std::vector<Eigen::Vector3d> posi_vec;
    std::vector<Eigen::Vector3d> gps_vec;
    std::vector<double> gps_time;
    double updated_gps_id=0;
    bool first_gps_updated=false;
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::NavSatFixPtr sgps = m.instantiate<sensor_msgs::NavSatFix>();
        if(sgps!=NULL){
            if((int)sgps->position_covariance[0]<=10){
                Eigen::Vector3d coor_gps;
                Eigen::Vector3d ori_gps;
                ori_gps(0)=sgps->latitude;
                ori_gps(1)=sgps->longitude;
                ori_gps(2)=sgps->altitude;
                if(gps_count==0){
                    anchorGps=ori_gps;
                }
                gps_count++;
                double timestamp = sgps->header.stamp.toSec();
                convert_to_coor(ori_gps, coor_gps, anchorGps);
                gps_vec.push_back(coor_gps);
                gps_time.push_back(timestamp);
            }
        }       
    }
    show_mp_as_cloud(gps_vec, "temp_gps");
    it= view.begin();
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL && is_init){
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(simg, "mono8");
            cv::Mat img_distort=cv_ptr->image;
            cv::Mat Img;
            cv::undistort(img_distort, Img, cam_inter_cv, cam_distort_cv);
            double msgTime = simg->header.stamp.toSec();
            if (msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_) {
                imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
            }
//             for(int i=updated_gps_id; i<gps_time.size(); i++){
//                 if(msgTime>gps_time[i]){
//                     if(gps_time[i]-msgTime<0.1){
//                         updated_gps_id=i;
//                         if(first_gps_updated==false){
//                             mpFilter_->safe_.state_.WrWM()=gps_vec[updated_gps_id];
//                             first_gps_updated=true;
//                         }else{
//                             poseUpdateMeas_.pos() = gps_vec[updated_gps_id];
//                             mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_, msgTime);
//                         }
//                     }
//                 }else{
//                     break;
//                 }
//             }
            
            
            imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[0].computeFromImage(Img, true);
            imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[0] = true;
            mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_, msgTime);
            imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
            img_count++;
        }
        
        sensor_msgs::ImuPtr simu = m.instantiate<sensor_msgs::Imu>();
        if(simu!=NULL){
            Eigen::Vector3d Accl(simu->linear_acceleration.x,
                         simu->linear_acceleration.y,
                         simu->linear_acceleration.z);
            Eigen::Vector3d Gyro(simu->angular_velocity.x,
                                simu->angular_velocity.y,
                                simu->angular_velocity.z);
            predictionMeas_.template get<mtPredictionMeas::_acc>() = Accl;
            predictionMeas_.template get<mtPredictionMeas::_gyr>() = Gyro;
            double time_s = simu->header.stamp.toSec();
            if(is_init==false){
                mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(), time_s);
                is_init=true;
            }else{
                mpFilter_->addPredictionMeas(predictionMeas_, time_s);
            }
        }
        if (simg!=NULL && is_init) {
            double lastImageTime;
            if (std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)) {
                mpFilter_->updateSafe(&lastImageTime);
            }
            Eigen::Matrix4d t_mb=Eigen::Matrix4d::Identity();
            t_mb.block(0,3,3,1)=mpFilter_->safe_.state_.WrWM();
            t_mb.block(0,0,3,3)=MPD(mpFilter_->safe_.state_.qWM()).matrix();
            posi_vec.push_back(mpFilter_->safe_.state_.WrWM().transpose());
            show_mp_as_cloud(posi_vec, "temp_kf");
            if(!mpFilter_->safe_.img_[0].empty()){
                visualization::RVizVisualizationSink::publish("match_img", mpFilter_->safe_.img_[0]);
            }
        }        
    }
    return 0;
}