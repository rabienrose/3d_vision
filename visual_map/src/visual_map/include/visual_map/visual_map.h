#pragma once
#include "visual_map/visual_map_common.h"

#include "visual_map/frame.h"
#include "visual_map/map_point.h"

namespace vm{
    class VisualMap{
    public:
        std::vector<std::shared_ptr<Frame>> frames;
        std::vector<std::shared_ptr<MapPoint>> mappoints;
    };
}
