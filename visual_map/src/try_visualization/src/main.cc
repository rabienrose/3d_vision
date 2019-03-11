#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

int main(int argc, char* argv[]) {
    visualization::RVizVisualizationSink::init();
    visualization::PoseVector poses_vis(10);
    const std::string& kNamespace = "vertices";
    visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, kNamespace, "before_opti");
}