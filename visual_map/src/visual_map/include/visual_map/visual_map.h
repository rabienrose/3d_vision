#pragma once
#include "visual_map/visual_map_common.h"

#include "visual_map/frame.h"
#include "visual_map/map_point.h"

namespace vm{
    class VisualMap{
    public:
        std::vector<std::shared_ptr<Frame>> frames;
        std::vector<std::shared_ptr<MapPoint>> mappoints;
        int map_id;
        Eigen::Vector3d gps_anchor;
        std::vector<std::shared_ptr<Frame>> pose_graph_v1;
        std::vector<std::shared_ptr<Frame>> pose_graph_v2;
        std::vector<Eigen::Vector3d> pose_graph_e_posi;
        std::vector<Eigen::Matrix3d> pose_graph_e_rot;
        std::vector<double> pose_graph_e_scale;

        void CreateSubMap(int startframe_id, int endframe_id, VisualMap& submap);
        void ComputeUniqueId();
        void DelMappoint(int id);
        void DelFrame(int id);
        void GetMPPosiList(std::vector<Eigen::Vector3d>& mp_posis);
        std::shared_ptr<vm::MapPoint> getMPById(int id);
        void CheckConsistence();
        void CheckLowQuaMappoint();
        void CheckLowQuaFrame();
    };
}
