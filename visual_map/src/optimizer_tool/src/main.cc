#include <string>
#include <fstream>
#include <memory>
#include "optimizer_tool/optimizer_tool.h"
#include "test_imu_tool/visual_tool.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(map_addr, "", "Folder of the map file, also the place to save the new map file.");
DEFINE_string(map_name, "", "File name of map file.");
DEFINE_string(opti_type, "BA", "Type of optimization: BA, Graph Opti, Visual Inertial Opi.");

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    if(FLAGS_opti_type=="BA"){
        OptimizerTool::optimize_gps_pose(FLAGS_map_addr, FLAGS_map_name);
    }else{
        OptimizerTool::optimize_sim3_graph(FLAGS_map_addr, FLAGS_map_name);
    }
    
}