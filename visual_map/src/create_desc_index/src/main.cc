#include <iostream>
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "create_desc_index.h"

int main(int argc, char** argv){
    visualization::RVizVisualizationSink::init();
    std::string resource_dir=argv[1];
    std::string map_name=argv[2];
    DescIndex::create_desc_index(resource_dir, map_name);
    
    return 0;
}
