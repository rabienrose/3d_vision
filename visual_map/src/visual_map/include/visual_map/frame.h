#pragma once

#include "visual_map/visual_map_common.h"
namespace vm{
    class MapPoint;
    class Frame {
    public:
        double time_stamp;
        std::string frame_file_name;
        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptors;
        std::vector<std::shared_ptr<MapPoint>> obss;
    };
}