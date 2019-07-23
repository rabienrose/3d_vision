#pragma once
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_common.h"
namespace vm {
void save_visual_map(VisualMap& map, std::string file_addr);
bool loader_visual_map(VisualMap& map, std::string file_addr);
}  // namespace vm
