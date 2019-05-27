#include "FindMatchBetweenFrames.h"
#include <glog/logging.h>


cv::Mat cam_matrix;
cv::Mat cam_distort;
bool get_camera_intrinsic(const std::string& calibration_file)
{
    cv::FileStorage fs{calibration_file, cv::FileStorage::READ};

    if (!fs.isOpened())
    {
        LOG(ERROR) << "Invalid calibration filename." ;
        return false;
    }

    fs["CameraMat"] >> cam_matrix;
    fs["DistCoeff"] >> cam_distort;

    return true;
}


int main(int argc, char* argv[])
{
    if(argc != 5)
    {
        LOG(ERROR) << "usage: find_match_lib_test bag_dir bag_name img_topic camera_yaml";
    }
    std::string bag_dir  = argv[1];
    std::string img_1 = argv[2];
    std::string img_2 = argv[3];
    std::string camera_yaml = argv[4];
    std::string img_name_1 = bag_dir + "/img/" +img_1; 
    std::string img_name_2 = bag_dir + "/img/" +img_2; 
    find_match::FindMatchBetweenFrames find_matches;
    if(!get_camera_intrinsic(bag_dir+"/"+camera_yaml)) return 0;
    find_matches.get_matches_from_images(img_name_1,img_name_2,cam_matrix,cam_distort);

}
