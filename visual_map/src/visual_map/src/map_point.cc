#include "visual_map/map_point.h"
#include "visual_map/frame.h"
namespace vm{

    void TrackItem::getUV(float& x, float& y, int& octave){
        if((size_t)kp_ind>=frame->kps.size()){
            std::cout<<"[TrackItem::getUV][error]kp_ind>=frame->kps.size()"<<std::endl;
            exit(0);
        }
        x = frame->kps[kp_ind].pt.x;
        y = frame->kps[kp_ind].pt.y;
        octave = frame->kps[kp_ind].octave;
    }
    
}