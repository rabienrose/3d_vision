#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include "bag_tool/extract_bag.h"

int main(int argc, char **argv){
    std::string bag_addr=argv[1];
    std::string out_dir=argv[2];
    std::string img_topic=argv[3];
    std::string imu_topic=argv[4];
    std::string gps_topic=argv[5];
    extract_bag(out_dir, bag_addr, img_topic, imu_topic, gps_topic);
    
}
