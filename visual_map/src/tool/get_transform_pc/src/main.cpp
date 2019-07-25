#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "orb_slam_lib/sim3_match.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::vector<Eigen::Vector3d> pc_1;
    pc_1.push_back(Eigen::Vector3d(180, 291, 0));
    pc_1.push_back(Eigen::Vector3d(179, 488, 0));
    pc_1.push_back(Eigen::Vector3d(463, 488, 0));
    std::vector<Eigen::Vector3d> pc_2;
    pc_2.push_back(Eigen::Vector3d(-1.5, -8.22, 0));
    pc_2.push_back(Eigen::Vector3d(-1.93, -19.62, 0));
    pc_2.push_back(Eigen::Vector3d(-18.16, -19.4, 0));
    double scale_12;
    Eigen::Matrix4d T12;
    orb_slam::ComputeSim3(pc_1, pc_2 , T12, scale_12);
    float avg_err=0;
    for(int i=0; i<pc_1.size(); i++){
        Eigen::Vector4d posi1_home;
        posi1_home.block(0,0,3,1)=pc_2[i];
        posi1_home(3)=1;
        Eigen::Vector4d posi2_home = T12*posi1_home;
        float err = (pc_1[i]-posi2_home.block(0,0,3,1)).norm();
        avg_err=avg_err+err/pc_1.size();
    }
    std::cout<<T12<<std::endl;
    std::cout<<scale_12<<std::endl;
    std::cout<<avg_err<<std::endl;
    return 0;
}