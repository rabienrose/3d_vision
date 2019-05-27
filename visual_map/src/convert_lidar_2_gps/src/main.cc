#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "read_write_data_lib/read_write.h"
#include <fstream>

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}

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
                                        
int main(int argc, char* argv[]){
    ros::init(argc, argv, "converter");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    std::vector<Eigen::Vector3d> lidar_posis;
    std::vector<Eigen::Quaterniond> lidar_dirs;
    std::vector<double> lidar_time;
    std::string lidar_addr=res_root+"/lidar_trajectory.txt";
    CHAMO::read_lidar_pose1(lidar_addr, lidar_dirs, lidar_posis, lidar_time);
    std::cout<<"lidar_addr: "<<lidar_posis.size()<<std::endl;
    
    std::string lidar_cam_addr=res_root+"/cam_with_lidar_posi.txt";
    std::string line;
    std::ifstream infile(lidar_cam_addr.c_str()); 
    Eigen::Vector3d cam_in_lidar_posi;
    if(infile.is_open()){
        std::getline(infile, line);
        std::vector<std::string> splited = split(line, ",");
        if(splited.size()==3){
            cam_in_lidar_posi(0)=atof(splited[0].c_str());
            cam_in_lidar_posi(1)=atof(splited[1].c_str());
            cam_in_lidar_posi(2)=atof(splited[2].c_str());
        }else{
            std::cout<<"image size reading wrong!!"<<std::endl;
            std::exit(0);
        }
    }else{
        std::cout<<"lidar_cam file open wrong: "<<lidar_cam_addr<<std::endl;
        std::exit(0);
    }
    std::cout<<"cam lidar: "<<cam_in_lidar_posi.transpose()<<std::endl;
    
    std::ofstream outfile_gps_orth;
    outfile_gps_orth.open (res_root+"/gps_orth.txt");
    outfile_gps_orth<<"31.1781323237559,121.609845297247,6.46666884233976"<<std::endl;
    show_mp_as_cloud(lidar_posis, "lidar_posi");
    std::vector<Eigen::Vector3d> cam_posis;
    for(int i=0; i<lidar_posis.size(); i++){
        Eigen::Matrix3d r(lidar_dirs[i]);
        Eigen::Vector3d cam_posi = r*cam_in_lidar_posi+lidar_posis[i];
        cam_posis.push_back(cam_posi);
        std::stringstream ss1;
        ss1<<std::setprecision (15)<<lidar_time[i]<<","<<cam_posi(0)<<","<<cam_posi(1)<<","<<cam_posi(2)<<", 1"<<std::endl; 
        outfile_gps_orth<<ss1.str();
    }
    show_mp_as_cloud(cam_posis, "cam_posi");
    outfile_gps_orth.close();
    //ros::spin();
    return 0;
}