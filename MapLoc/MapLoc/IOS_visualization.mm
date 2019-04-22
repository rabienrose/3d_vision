#import "IOS_visualization.h"
#include "read_write_data_lib/read_write.h"
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>
@interface IOSVis ()

@end
@implementation IOSVis
+ (void)showMPs: (std::string) mp_addr sceneDelegate: (id<SceneInfoDelegate>) sceneDelegate{
    std::vector<Eigen::Vector3d> mp_posis;
    CHAMO::read_mp_posi(mp_addr, mp_posis);
//    for(int i=0; i<mp_posis.size(); i++){
//        mp_posis[i] = Rwi*mp_posis[i];
//    }
    if(mp_posis.size()>0){
        [sceneDelegate showPC: mp_posis];
    }
}

+ (void)showTraj: (std::string) traj_addr sceneDelegate: (id<SceneInfoDelegate>) sceneDelegate{
    std::vector<Eigen::Vector3d> traj;
    std::string line;
    std::ifstream infile_pose(traj_addr.c_str());
    while (true)
    {
        std::getline(infile_pose, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = CHAMO::split(line, ",");
        std::string file_name= splited[0];
        Eigen::Vector3d posi;
        posi(0)=atof(splited[5].c_str());
        posi(1)=atof(splited[9].c_str());
        posi(2)=atof(splited[13].c_str());
        traj.push_back(posi);
    }
//    for(int i=0; i<traj.size(); i++){
//        traj[i] = Rwi*traj[i];
//    }
    if(traj.size()>0){
        [sceneDelegate showTraj: traj];
    }
}

@end
