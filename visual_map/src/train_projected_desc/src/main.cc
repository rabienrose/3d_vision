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
    std::string track_addr="/home/chamo/Documents/work/orb_mapping_loc/track.txt";
    std::string desc_addr="/home/chamo/Documents/work/orb_mapping_loc/desc.txt";
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
    std::vector<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> > raw_descs;
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
        raw_descs.push_back(desc);
        Eigen::MatrixXf desc_eigen;
        get_eigen_desc(desc, desc_eigen);
        append_eigen_desc(global_descs, desc_eigen);
    }
    
    if(true){
        std::ifstream in_stream("/home/chamo/Documents/work/orb_mapping_loc/orb_project_mat.dat", std::ios_base::binary);
        Eigen::MatrixXf projection_matrix_;
        common::Deserialize(&projection_matrix_, &in_stream);
        std::vector<descriptor_projection::DescriptorMatch> matches;
        std::vector<descriptor_projection::DescriptorMatch> non_matches;
        descriptor_projection::BuildListOfMatchesAndNonMatches(global_descs, tracks, &matches, &non_matches);
        float mean_diff_match=0;
        float max_diff_match=0;
        int outlier_match_count=0;
        for(int i=0; i<matches.size(); i++){
            Eigen::VectorXf projected_descriptor1;
            descriptor_projection::ProjectDescriptor(raw_descs[matches[i].first], projection_matrix_, 10, projected_descriptor1);
            Eigen::VectorXf projected_descriptor2;
            descriptor_projection::ProjectDescriptor(raw_descs[matches[i].second], projection_matrix_, 10, projected_descriptor2);
            float diff = (projected_descriptor1-projected_descriptor2).norm();
            //std::cout<<(projected_descriptor1-projected_descriptor2).norm()<<std::endl;
            mean_diff_match=mean_diff_match+diff/matches.size();
            if (diff>max_diff_match){
                max_diff_match=diff;
            }
            if (diff>3){
                outlier_match_count++;
            }
            
        }
        std::cout<<"mean_diff_match: "<<mean_diff_match<<std::endl;
        std::cout<<"max_diff_match: "<<max_diff_match<<std::endl;
        std::cout<<"outlier_match_count: "<<outlier_match_count/(float)matches.size()<<std::endl;
        float mean_diff_nonmatch=0;
        float min_diff_match=999;
        int outlier_nonmatch_count=0;
        for(int i=0; i<non_matches.size(); i++){
            Eigen::VectorXf projected_descriptor1;
            descriptor_projection::ProjectDescriptor(raw_descs[non_matches[i].first], projection_matrix_, 10, projected_descriptor1);
            Eigen::VectorXf projected_descriptor2;
            descriptor_projection::ProjectDescriptor(raw_descs[non_matches[i].second], projection_matrix_, 10, projected_descriptor2);
            float diff = (projected_descriptor1-projected_descriptor2).norm();
            //std::cout<<(projected_descriptor1-projected_descriptor2).norm()<<std::endl;
            mean_diff_nonmatch=mean_diff_nonmatch+diff/non_matches.size();
            if (diff<min_diff_match){
                min_diff_match=diff;
            }
            if (diff<3){
                outlier_nonmatch_count++;
            }
        }
        std::cout<<"mean_diff_nonmatch: "<<mean_diff_nonmatch<<std::endl;
        std::cout<<"min_diff_match: "<<min_diff_match<<std::endl;
        std::cout<<"outlier_nonmatch_count: "<<outlier_nonmatch_count/(float)non_matches.size()<<std::endl;
        return 0;
    }

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
    std::ofstream out_stream("/home/chamo/Documents/work/orb_mapping_loc/orb_project_mat.dat", std::ios_base::binary);
    common::Serialize(A, &out_stream);
    out_stream.close();
}