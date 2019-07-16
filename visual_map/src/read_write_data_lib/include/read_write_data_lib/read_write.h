#pragma once
#include <Eigen/Dense>
#include <string>
#include <map>
#include <vector>

namespace CHAMO
{
    void read_pose_list(std::map<double, int>& pose_list, std::map<int, int>& frame_ids,
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& pose_vec,
        std::vector<double>& img_times, 
        std::string pose_addr, std::string img_time_addr
    );
    
    void read_traj_file(std::string traj_file_addr, std::vector<Eigen::Matrix4d>& traj_out, std::vector<std::string>& frame_names);
    
    void read_img_time(std::string img_time_addr, std::vector<double>& img_timess, std::vector<std::string>& img_names);
    void read_gps_orth(std::string gps_orth_addr, std::vector<Eigen::Vector3d>& gps_orths, std::vector<double>& gps_times,
        std::vector<int>& gps_covs, Eigen::Vector3d& anchor_gps
    );
    
    void read_imu_data(std::string imu_addr, std::vector<Eigen::Matrix<double, 7, 1>>& imu_datas);
    
    void read_mp_posi(std::string posi_addr, std::vector<Eigen::Vector3d>& mp_posis);
    
    void read_kp_info(std::string kp_addr, std::vector<Eigen::Vector2f>& kp_uvs, std::vector<std::string>& kp_framenames, std::vector<int>& kp_octoves);
    
    void read_track_info(std::string track_addr, std::vector<std::vector<int>>& tracks);
    
    void read_lidar_pose(std::string lidar_addr, std::vector<Eigen::Quaterniond>& lidar_dirs,
        std::vector<Eigen::Vector3d>& lidar_posis,
        std::vector<double>& time_stamp
    );
    
    void read_desc_eigen(std::string desc_addr, std::vector<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>>& descs);
    void read_cam_info(std::string cam_addr, Eigen::Matrix3d& cam_inter, Eigen::Vector4d& cam_distort, Eigen::Matrix4d& Tbc);
    
    void read_lidar_pose1(std::string lidar_addr, std::vector<Eigen::Quaterniond>& lidar_dirs,
        std::vector<Eigen::Vector3d>& lidar_posis,
        std::vector<double>& time_stamp
    );
    std::vector<std::string> split(const std::string& str, const std::string& delim);
    void read_gps_alin(std::string gps_alin_addr, std::vector<Eigen::Vector3d>& gps_alins, std::vector<int>& inliars, std::vector<float>& accus);
    void read_image_info(std::string image_addr, int& width, int& height, float& desc_scale, int& desc_level, int& desc_count);
}
