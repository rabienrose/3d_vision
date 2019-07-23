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

void findNearGPS(int& gps1, int& gps2, std::vector<double>& gps_times, double frame_time){
    gps1=-1;
    gps2=-1;
    for(int i=0; i<gps_times.size() ; i++){
        if(gps_times[i]>frame_time){
            if(i==0){
                return;
            }
            if(frame_time - gps_times[i-1]>2){
                return;
            }
            if(gps_times[i] - frame_time >2){
                return;
            }
            gps1=i-1;
            gps2=i;
            return;
        }
    }
}

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}
                                        
int main(int argc, char* argv[]){
    ros::init(argc, argv, "converter");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    double anchor_x=atof(argv[2]);
    double anchor_y=atof(argv[3]);
    double anchor_z=atof(argv[4]);
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
    
    std::string img_time_addr=res_root+"/image_time.txt";
    std::vector<double> img_timess;
    std::vector<std::string> img_names;
    CHAMO::read_img_time(img_time_addr, img_timess, img_names);
    std::cout<<"img_timess: "<<img_timess.size()<<std::endl;
    
    std::ofstream outfile_gps_orth;
    outfile_gps_orth.open (res_root+"/gps_orth.txt");
    
    std::vector<double> gps_times_hcov;
    std::vector<Eigen::Vector3d> gps_orths_hcov;
    std::vector<double> gps_confid;
    std::vector<double> new_gps_confid;
    
    std::vector<Eigen::Vector3d> cam_posis;
    
    Eigen::Vector3d anchor_point;
    for(int i=0; i<lidar_posis.size(); i++){
        Eigen::Matrix3d r(lidar_dirs[i]);
        Eigen::Vector3d cam_posi = r*cam_in_lidar_posi+lidar_posis[i];
        if(i==0){
            if(anchor_x==0 && anchor_x==0 && anchor_x==0){
                anchor_point=cam_posi;
            }else{
                anchor_point(0)=anchor_x;
                anchor_point(1)=anchor_y;
                anchor_point(2)=anchor_z;
            }
            std::cout<<std::setprecision (15)<<"anchor: "<<anchor_point.transpose()<<std::endl;
            outfile_gps_orth<<std::setprecision (15)<<cam_posi(0)<<","<<cam_posi(1)<<","<<cam_posi(2)<<std::endl;
        }
        cam_posis.push_back(cam_posi);
        std::stringstream ss1;
        ss1<<std::setprecision (15)<<lidar_time[i]<<","<<cam_posi(0)-anchor_point(0)<<","<<cam_posi(1)-anchor_point(1)<<","<<cam_posi(2)-anchor_point(2)<<", 1"<<std::endl; 
        gps_times_hcov.push_back(lidar_time[i]);
        gps_orths_hcov.push_back(cam_posi-anchor_point);
        gps_confid.push_back(1);
        outfile_gps_orth<<ss1.str();
    }
    show_mp_as_cloud(cam_posis, "cam_posi");
    outfile_gps_orth.close();
    
    std::vector<int> img_to_gps_ids;
    std::vector<Eigen::Vector3d> img_gpss;
    for (int i=0; i<img_timess.size(); i++){
        int gps1;
        int gps2;
        findNearGPS(gps1, gps2, gps_times_hcov, img_timess[i]);
        
        if(gps1== -1){
            img_to_gps_ids.push_back(-1);
            continue;
        }
        double i_gps_x;
        double i_gps_y;
        double i_gps_z;
        double inter_confid;
        interDouble(gps_orths_hcov[gps1](0), gps_orths_hcov[gps2](0), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_x, img_timess[i]);
        interDouble(gps_orths_hcov[gps1](1), gps_orths_hcov[gps2](1), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_y, img_timess[i]);
        interDouble(gps_orths_hcov[gps1](2), gps_orths_hcov[gps2](2), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_z, img_timess[i]);
        interDouble(gps_confid[gps1], gps_confid[gps2], gps_times_hcov[gps1], gps_times_hcov[gps2], inter_confid, img_timess[i]);
        
        Eigen::Vector3d new_gps_frame;
        new_gps_frame(0)=i_gps_x;
        new_gps_frame(1)=i_gps_y;
        new_gps_frame(2)=i_gps_z;
        img_gpss.push_back(new_gps_frame);
        new_gps_confid.push_back(inter_confid);
        img_to_gps_ids.push_back(img_gpss.size()-1);
    }
    std::string gps_out_addr=res_root+"/gps_alin.txt";
    std::ofstream outfile_gps_align;
    outfile_gps_align.open(gps_out_addr.c_str());
    for(int i=0; i<img_timess.size(); i++){
        int gps_id = img_to_gps_ids[i];
        if(gps_id==-1){
            outfile_gps_align<<img_names[i]<<","<<"-1"
            <<std::endl;
        }else{
            Eigen::Vector3d gps_temp= img_gpss[gps_id];
            outfile_gps_align<<img_names[i]<<","<<i
            <<","<<gps_temp(0)<<","<<gps_temp(1)<<","<<gps_temp(2)<<","<<(int)new_gps_confid[gps_id]
            <<std::endl;
        }
    }
    outfile_gps_align.close();
    //ros::spin();
    return 0;
}