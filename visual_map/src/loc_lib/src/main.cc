#include <loc_lib/ChamoLoc.h>

#include "orb_slam_lib/two_frame_pose.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"


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
    visualization::RVizVisualizationSink::init();
    wayz::ChamoLoc localizer;
    localizer.StartLocalization("/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/try/rovio_default_config.info");
    localizer.AddMap("/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/try");
    
    std::string img_time_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/try/camera_1_image_time.txt";
    std::ifstream infile_img_time(img_time_addr.c_str()); 
    std::unordered_map<std::string, double> img_time_map;
    std::string line;
    while (true)
    {
        std::getline(infile_img_time, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        img_time_map[splited[0]]=atof(splited[1].c_str());
    }
    
    std::string imu_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/orb_slam_re/try/imu.txt";
    std::ifstream infile_imu(imu_addr.c_str()); 
    std::vector<double> imu_time;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> acc_data;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gyro_data;
    while (true)
    {
        std::getline(infile_imu, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        imu_time.push_back(atof(splited[0].c_str()));
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        gyro(0)=atof(splited[1].c_str());
        gyro(1)=atof(splited[2].c_str());
        gyro(2)=atof(splited[3].c_str());
        acc(0)=atof(splited[4].c_str());
        acc(1)=atof(splited[5].c_str());
        acc(2)=atof(splited[6].c_str());
        acc_data.push_back(acc);
        gyro_data.push_back(gyro);
    }
    
    std::string img_root="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/camera_1_img/";
    int imu_index=0;
    for(int n=0; n<4010 ;n=n+1){
        
        std::stringstream ss;
        ss<<"img_"<<n<<".jpg";
        double img_time = img_time_map[ss.str()];
        cv::Mat img = cv::imread(img_root+ss.str());
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        localizer.AddImage(img_time,0, img);
        while(imu_time[imu_index]<img_time){
            //std::cout<<std::setprecision(15)<<imu_time[imu_index]<<" , "<<img_time<<std::endl;
            localizer.AddIMU(imu_time[imu_index], acc_data[imu_index], gyro_data[imu_index]);
            imu_index++;
        }
//         Eigen::Vector3d Pos;
//         Eigen::Vector3d Vel;
//         Eigen::Quaterniond Ori;
//         bool re = localizer.QueryPose(img_time, Pos, Vel, Ori);
//         if(re ==true){
//             std::cout<<n<<" : "<<Pos.transpose()<<std::endl;
//         }
        ros::spinOnce();
        
    }
    return 0;
}