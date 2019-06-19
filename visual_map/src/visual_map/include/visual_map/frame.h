#pragma once

#include "visual_map/visual_map_common.h"
namespace vm{
    class MapPoint;
    class Frame {
    public:
        int id;
        double time_stamp;
        std::string frame_file_name;
        std::vector<cv::KeyPoint> kps;
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> descriptors;
        std::vector<std::shared_ptr<MapPoint>> obss;
        Eigen::Vector3d position;
        Eigen::Quaterniond direction;
        Eigen::Vector3d gps_position;
        float gps_accu;
        
        float fx;
        float fy;
        float cx;
        float cy;
        float k1; //radian
        float k2; //radian
        float p1; //tan
        float p2; //tan
        int width;
        int height;
        
        Eigen::Matrix4d getPose();
        void setPose(Eigen::Matrix4d pose);
        
        Eigen::Matrix<double,3, 3> getKMat();
        Eigen::Matrix<double,3, 4> getProjMat();
        
        void getDesc(int ind, Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& desc_out);
        
    };
}