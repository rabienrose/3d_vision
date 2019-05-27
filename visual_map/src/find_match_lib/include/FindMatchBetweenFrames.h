#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <memory>
#include "read_write_data_lib/read_write.h"
#include "orb_slam_lib/two_frame_pose.h"
#include "orb_slam_lib/DescExtractor.h"
#include "orb_slam_lib/DescMatcher.h"
#include "ORBextractor.h"
namespace find_match {

struct KpInfo{
  KpInfo(cv::KeyPoint *kp, cv::Mat desc, int frame_id, int track_id,
         bool is_key_frame)
      : kp(kp), desc(desc), frame_id(frame_id), track_id(track_id),
        is_key_frame(is_key_frame){};
  cv::KeyPoint *kp;
  cv::Mat desc;
  int frame_id;
  int track_id;
  // int kp_id;
  bool is_key_frame;
};

struct KFInfo{
    int frame_id;
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
};

typedef std::vector<std::vector<std::vector<std::size_t>>> mgrid_info;
  
class FindMatchBetweenFrames{
    public:
        FindMatchBetweenFrames();

        ~FindMatchBetweenFrames();

        void get_matches_from_bag(std::string& bag_dir, std::string& bag_file_name, 
                                  std::string& img_topic, cv::Mat cam_matrix, cv::Mat cam_distort);
        void get_matches_from_images(std::string& image_1, std::string& image_2, 
                                  cv::Mat cam_matrix, cv::Mat cam_distort);

        void find_tracks();

        void extract_orb(cv::Mat img, cv::Mat mask, cv::Mat& desc_list,
                         std::vector<cv::KeyPoint>& kps_list, mgrid_info& mGrid);

        void write_tracks(std::string& output_dir);

        void get_pose(std::string& output_dir);

        bool find_nearest_key_frame(int frame_id,int& key_frame_id);
    private:
        std::vector<std::vector<cv::KeyPoint>> kps_list_in_all_frames;
        std::vector<cv::Mat> desc_list_in_all_frames;
        std::vector<mgrid_info> mgrid_in_all_frames;
        
        int width;
        int height; 
        cv::Mat cam_mat;
        std::string dir_name;

    public:
        std::vector<std::pair<int, std::vector<std::shared_ptr<KpInfo>>>> kps_info_list_in_all_frames;         //save all kp info in every frame
        std::map<int, std::vector<std::shared_ptr<KpInfo>>> kp_track_pairs; //save kp info which is in a track
        std::vector<KFInfo> key_frame_poses;
};  
}
