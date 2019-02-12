#pragma once
#include "visual_map/visual_map_common.h"

namespace vm{
    class Frame;
    class TrackItem{
    public:
        std::shared_ptr<Frame> frame;
        int kp_ind;
    };
    
    class MapPoint{
    public:
        Eigen::Vector3d position;
        std::vector<TrackItem> track;
    };

}