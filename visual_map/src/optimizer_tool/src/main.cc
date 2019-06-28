#include <string>
#include <fstream>
#include <memory>
#include "optimizer_tool/optimizer_tool.h"
#include "test_imu_tool/visual_tool.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"


int main(int argc, char* argv[]) {
    std::string map_addr=argv[1];
    std::string map_name=argv[2];
    std::string opti_type=argv[3];
    visualization::RVizVisualizationSink::init();
    if(opti_type=="BA"){
        OptimizerTool::optimize_gps_pose(map_addr, map_name);
    }else{
        OptimizerTool::optimize_sim3_graph(map_addr, map_name);
    }
    
}