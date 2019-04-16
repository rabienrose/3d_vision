#include <string>
#include <fstream>
#include <memory>
#include "optimizer_tool/optimizer_tool.h"
#include "test_imu_tool/visual_tool.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"

int main(int argc, char* argv[]) {
    std::string res_root=argv[1];
    visualization::RVizVisualizationSink::init();
    OptimizerTool::optimize_imu(res_root);
    OptimizerTool::optimize_true_pose(res_root);
    ros::spin();
}