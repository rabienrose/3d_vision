#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "ORBVocabulary.h"

namespace ORB_SLAM2 {
    class KeyFrameDatabase;
    class Map;
    class Frame;
}

namespace chamo {
        void visMap(ORB_SLAM2::Map* mpMap);
        Eigen::Matrix4d MatchWithGlobalMap(ORB_SLAM2::Frame& frame, 
                                 ORB_SLAM2::ORBVocabulary* mpVocabulary, 
                                 ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase, 
                                 ORB_SLAM2::Map* mpMap);
        void LoadORBMap(std::string res_root, 
                                 ORB_SLAM2::ORBVocabulary*& mpVocabulary, 
                                 ORB_SLAM2::KeyFrameDatabase*& mpKeyFrameDatabase, 
                                 ORB_SLAM2::Map*& mpMap
                       );
        void GetORBextractor(std::string res_root, ORB_SLAM2::ORBextractor*& mpORBextractor, cv::Mat& mK, cv::Mat& mDistCoef);
        void GetAFrame(cv::Mat img, ORB_SLAM2::Frame& frame, ORB_SLAM2::ORBextractor* mpORBextractor, ORB_SLAM2::ORBVocabulary*& mpVocabulary,
                       cv::Mat mK, cv::Mat mDistCoef, std::string file_name, double timestamp);
}