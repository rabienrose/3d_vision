#include <string>
#include <fstream>
#include <memory>

#include "chamo_map.pb.h"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "input arg number not right" << std::endl;
        return -1;
    }
    std::string save_addr = argv[1];
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    std::shared_ptr<chamo::MapData> map_data_ptr;
    map_data_ptr.reset(new chamo::MapData);
    map_data_ptr->set_name("chamo");
    for (int i=0; i<3; i++){
        chamo::MapData::Child* child = map_data_ptr->add_children();
        child->set_child_name("lili");
        child->set_child_id(i);
    }
    
    std::fstream output(save_addr.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
    
    if (!map_data_ptr->SerializeToOstream(&output)) {
        std::cerr << "Failed to write map data." << std::endl;
        return -1;
    }
    
    output.close();
    
    std::fstream input(save_addr.c_str(), std::ios::in | std::ios::binary);
    
    chamo::MapData map_data_in;
    if (!map_data_in.ParseFromIstream(&input)) {
        std::cerr << "Failed to parse map data." << std::endl;
        return -1;
    }
    
    std::cout<<map_data_in.has_name()<<std::endl;
    std::cout<<map_data_in.children_size()<<std::endl;
    std::cout<<map_data_in.children(2).child_id()<<std::endl;

    input.close();

    return 0;
}