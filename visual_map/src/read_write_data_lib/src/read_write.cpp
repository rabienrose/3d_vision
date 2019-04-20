#include "read_write_data_lib/read_write.h"
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>

namespace CHAMO
{
    std::vector<std::string> split(const std::string& str, const std::string& delim)
    {
        std::vector<std::string> tokens;
        size_t prev = 0, pos = 0;
        do
        {
            pos = str.find(delim, prev);
            if (pos == std::string::npos) pos = str.length();
            std::string token = str.substr(prev, pos-prev);
            if (!token.empty()) tokens.push_back(token);
            prev = pos + delim.length();
        }
        while (pos < str.length() && prev < str.length());
        return tokens;
    }
    
    void read_cam_info(std::string cam_addr, Eigen::Matrix3d& cam_inter, Eigen::Vector4d& cam_distort, Eigen::Matrix4d& Tbc){
        std::string line;
        std::ifstream infile_camera(cam_addr.c_str()); 
        std::getline(infile_camera, line);
        std::vector<std::string> splited = split(line, ",");
        cam_inter=Eigen::Matrix3d::Identity();
        cam_inter(0,0)=atof(splited[0].c_str());
        cam_inter(1,1)=atof(splited[1].c_str());
        cam_inter(0,2)=atof(splited[2].c_str());
        cam_inter(1,2)=atof(splited[3].c_str());
        cam_distort(0)=atof(splited[4].c_str());
        cam_distort(1)=atof(splited[5].c_str());
        cam_distort(2)=atof(splited[6].c_str());
        cam_distort(3)=atof(splited[7].c_str());
        Tbc=Eigen::Matrix4d::Identity();
        std::getline(infile_camera, line);
        splited = split(line, ",");
        Tbc(0,0)=atof(splited[0].c_str());
        Tbc(0,1)=atof(splited[1].c_str());
        Tbc(0,2)=atof(splited[2].c_str());
        Tbc(0,3)=atof(splited[3].c_str());
        Tbc(1,0)=atof(splited[4].c_str());
        Tbc(1,1)=atof(splited[5].c_str());
        Tbc(1,2)=atof(splited[6].c_str());
        Tbc(1,3)=atof(splited[7].c_str());
        Tbc(2,0)=atof(splited[8].c_str());
        Tbc(2,1)=atof(splited[9].c_str());
        Tbc(2,2)=atof(splited[10].c_str());
        Tbc(2,3)=atof(splited[11].c_str());
    }
    
    void read_pose_list(std::map<double, int>& pose_list, std::map<int, int>& frame_ids,
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& pose_vec,
        std::vector<double>& img_times, 
        std::string pose_addr, std::string img_time_addr
    ){
        std::string line;
        std::unordered_map<std::string, double> img_time_map;
        if(img_time_addr!=""){
            std::ifstream infile_img_time(img_time_addr.c_str()); 
            while (true)
            {
                std::getline(infile_img_time, line);
                if (line==""){
                    break;
                }
                std::vector<std::string> splited = split(line, ",");
                img_time_map[splited[0]]=atof(splited[1].c_str());
            }
        }

        std::ifstream infile_pose(pose_addr.c_str());
        while (true)
        {
            std::getline(infile_pose, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            std::string file_name= splited[0];
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            int frame_id=atoi(splited[1].c_str());
            pose(0,0)=atof(splited[2].c_str());
            pose(0,1)=atof(splited[3].c_str());
            pose(0,2)=atof(splited[4].c_str());
            pose(0,3)=atof(splited[5].c_str());
            pose(1,0)=atof(splited[6].c_str());
            pose(1,1)=atof(splited[7].c_str());
            pose(1,2)=atof(splited[8].c_str());
            pose(1,3)=atof(splited[9].c_str());
            pose(2,0)=atof(splited[10].c_str());
            pose(2,1)=atof(splited[11].c_str());
            pose(2,2)=atof(splited[12].c_str());
            pose(2,3)=atof(splited[13].c_str());
            pose_vec.push_back(pose);
            
            frame_ids[frame_id]=pose_vec.size()-1;
            if(img_time_map.size()>0){
                if(img_time_map.count(file_name)!=0){
                    pose_list[img_time_map[file_name]]=pose_vec.size()-1;
                    img_times.push_back(img_time_map[file_name]);
                }else{
                    std::cout<<"image not exist!!"<<std::endl;
                    return;
                }
            }      
        }
    }
    
    void read_imu_data(std::string imu_addr, std::vector<Eigen::Matrix<double, 7, 1>>& imu_datas){
        std::string line;
        std::ifstream infile_imu(imu_addr.c_str());
        int imu_count=0;
        Eigen::Vector3d gravity(0,0,-9.8);
        Eigen::Vector3d bg=Eigen::Vector3d::Zero();
        Eigen::Vector3d ba=Eigen::Vector3d::Zero();
        
        while (true)
        {
            std::getline(infile_imu, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            Eigen::Matrix<double, 7, 1> imu;
            for(int i=0; i<7; i++){
                imu(i)=atof(splited[i].c_str());
            }
            imu_datas.push_back(imu);
        }
    }
    
    void read_mp_posi(std::string posi_addr, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& mp_posis){
        std::string line;
        std::ifstream infile_posi(posi_addr.c_str());
        while (true)
        {
            std::getline(infile_posi, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            Eigen::Vector3d mp_posi;
            mp_posi.x()=atof(splited[0].c_str());
            mp_posi.y()=atof(splited[1].c_str());
            mp_posi.z()=atof(splited[2].c_str());
            mp_posis.push_back(mp_posi);
        }
    }
    
    void read_kp_info(std::string kp_addr, std::vector<Eigen::Vector2f>& kp_uvs, std::vector<int>& kp_frameids, std::vector<int>& kp_octoves){
        std::string line;
        std::ifstream infile_kp(kp_addr.c_str());
        while (true)
        {
            std::getline(infile_kp, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            Eigen::Vector2f uv;
            uv.x()=atof(splited[0].c_str());
            uv.y()=atof(splited[1].c_str());
            int octove=atoi(splited[2].c_str());
            int frame_id=atoi(splited[3].c_str());
            kp_uvs.push_back(uv);
            kp_octoves.push_back(octove);
            kp_frameids.push_back(frame_id);
        }
    }
    
    void read_track_info(std::string track_addr, std::vector<std::vector<int>>& tracks){
        std::string line;
        std::ifstream infile_track(track_addr);
        while (true)
        {
            std::getline(infile_track, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            std::vector<int> track;
            for(auto str: splited){
                track.push_back(atoi(str.c_str()));
            }
            tracks.push_back(track);
        }
    }
    
    void read_lidar_pose(std::string lidar_addr, std::vector<Eigen::Quaterniond>& lidar_dirs,
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& lidar_posis,
        std::vector<double>& time_stamp
    ){
        std::string line;
        std::ifstream infile_lidar(lidar_addr);
        while (true)
        {
            std::getline(infile_lidar, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            Eigen::Vector3d posi;
            posi.x()=atof(splited[3].c_str());
            posi.y()=atof(splited[4].c_str());
            posi.z()=atof(splited[5].c_str());
            Eigen::Quaterniond qua;
            qua.w()=atof(splited[6].c_str());
            qua.x()=atof(splited[7].c_str());
            qua.y()=atof(splited[8].c_str());
            qua.z()=atof(splited[9].c_str());
            double time= atof(splited[2].c_str());
            time_stamp.push_back(time);
            lidar_posis.push_back(posi);
            lidar_dirs.push_back(qua);
        }
    }
    
    void read_lidar_pose1(std::string lidar_addr, std::vector<Eigen::Quaterniond>& lidar_dirs,
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& lidar_posis,
        std::vector<double>& time_stamp
    ){
        std::string line;
        std::ifstream infile_lidar(lidar_addr);
        while (true)
        {
            std::getline(infile_lidar, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            Eigen::Vector3d posi;
            posi.x()=atof(splited[6].c_str());
            posi.y()=atof(splited[10].c_str());
            posi.z()=atof(splited[14].c_str());
            Eigen::Matrix3d rot;
            rot(0,0)=atof(splited[3].c_str());
            rot(0,1)=atof(splited[4].c_str());
            rot(0,2)=atof(splited[5].c_str());
            rot(1,0)=atof(splited[7].c_str());
            rot(1,1)=atof(splited[8].c_str());
            rot(1,2)=atof(splited[9].c_str());
            rot(2,0)=atof(splited[11].c_str());
            rot(2,1)=atof(splited[12].c_str());
            rot(2,2)=atof(splited[13].c_str());
            double time= atof(splited[2].c_str());
            lidar_dirs.push_back(Eigen::Quaterniond(rot));
            time_stamp.push_back(time);
            lidar_posis.push_back(posi);
        }
    }
    
    void read_desc_eigen(std::string desc_addr, std::vector<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>>& descs){
        std::string line;
        std::ifstream infile_desc(desc_addr);
        while (true)
        {
            std::getline(infile_desc, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> desc;
            desc.conservativeResize(splited.size(),1);
            for(int i=0; i<splited.size(); i++){
                desc(i,0)=(unsigned char)atoi(splited[i].c_str());
            }
            descs.push_back(desc);
        }
    }
}
