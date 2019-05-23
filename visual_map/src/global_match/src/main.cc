#include <global_match.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
                                        
int main(int argc, char* argv[]){
    ros::init(argc, argv, "loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_addr=argv[1];
    

    return 0;
}