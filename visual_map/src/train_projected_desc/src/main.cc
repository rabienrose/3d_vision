#include <string>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/eigen.hpp>
#include <random>
#include <math.h>
#include "descriptor-projection/build-projection-matrix.h"
#include <maplab-common/binary-serialization.h>

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

void get_eigen_desc(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> desc_eigen_temp, Eigen::MatrixXf& desc_eigen ){
        desc_eigen.resize(desc_eigen_temp.rows()*8, 1);
        descriptor_projection::DescriptorToEigenMatrix(desc_eigen_temp, desc_eigen);
    }

void append_eigen_desc(Eigen::MatrixXf& desc_eigen, Eigen::MatrixXf& new_desc_eigen ){
    int old_count=desc_eigen.cols();
    desc_eigen.conservativeResize(new_desc_eigen.rows(), old_count+new_desc_eigen.cols());
    desc_eigen.block(0, old_count, new_desc_eigen.rows(), new_desc_eigen.cols())=new_desc_eigen;
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    std::string track_addr="/home/chamo/Documents/work/orb_mapping_loc/track_desc/track.txt";
    std::string desc_addr="/home/chamo/Documents/work/orb_mapping_loc/track_desc/desc.txt";
    std::vector<std::vector<int>> tracks;

    Eigen::MatrixXf global_descs;
    std::ifstream infile(track_addr);
    std::string line;
    while (true)
    {
        std::getline(infile, line);
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
    
    std::ifstream infile_desc(desc_addr);;
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
        Eigen::MatrixXf desc_eigen;
        get_eigen_desc(desc, desc_eigen);
        append_eigen_desc(global_descs, desc_eigen);
    }

    
    std::vector<descriptor_projection::DescriptorMatch> matches;
    std::vector<descriptor_projection::DescriptorMatch> non_matches;
    descriptor_projection::BuildListOfMatchesAndNonMatches(global_descs, tracks, &matches, &non_matches);
    std::cout<<"match count: "<<matches.size()<<" | non match count: "<<non_matches.size()<<std::endl;
    unsigned int sample_size_matches;
    unsigned int sample_size_non_matches;
    Eigen::MatrixXf cov_matches;
    Eigen::MatrixXf cov_non_matches;
    descriptor_projection::BuildCovarianceMatricesOfMatchesAndNonMatches(global_descs.rows(), global_descs, tracks, 
                                                  &sample_size_matches, &sample_size_non_matches,
                                                  &cov_matches, &cov_non_matches
                                                 );
    Eigen::MatrixXf A;
    descriptor_projection::ComputeProjectionMatrix(cov_matches, cov_non_matches, &A);
    std::cout<<A<<std::endl;
    std::ofstream out_stream("/home/chamo/Documents/work/orb_mapping_loc/track_desc/orb_project_mat.dat", std::ios_base::binary);
    common::Serialize(A, &out_stream);
    out_stream.close();
}