#include <algorithm>
#include <bitset>
#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <aslam/common/feature-descriptor-ref.h>
#include <aslam/frames/visual-frame.h>
#include "vocabulary-tree/tree-builder.h"
#include "vocabulary-tree/types.h"
#include "descriptor-projection/build-projection-matrix.h"
#include <maplab-common/binary-serialization.h>

DEFINE_int32(
    lc_kmeans_splits, 10, "Number of splits in the kmeans step per level.");

DEFINE_int32(lc_kmeans_levels, 4, "Number of levels in the vocabulary tree.");

DEFINE_int32(lc_num_kmeans_restarts, 5, "Number of restarts for the kmeans.");

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

void append_eigen_desc(Eigen::MatrixXf& desc_eigen, Eigen::MatrixXf& new_desc_eigen ){
    int old_count=desc_eigen.cols();
    desc_eigen.conservativeResize(new_desc_eigen.rows(), old_count+new_desc_eigen.cols());
    desc_eigen.block(0, old_count, new_desc_eigen.rows(), new_desc_eigen.cols())=new_desc_eigen;
}

int main(int argc, char* argv[]) {
  
    typedef aslam::common::FeatureDescriptorRef desc_type;
    typedef loop_closure::TreeBuilder<desc_type, loop_closure::distance::Hamming<desc_type> > TreeBuilderChamo;
    TreeBuilderChamo::FeatureVector descriptor_refs;
    std::string desc_addr="/home/chamo/Documents/work/orb_mapping_loc/track_desc/desc.txt";
    std::ifstream infile_desc(desc_addr);
    std::string line;
    int descriptor_size= -1;
    std::vector<aslam::VisualFrame::DescriptorsT> desc_eigen_list;
    while (true)
    {
        std::getline(infile_desc, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        
        
        aslam::VisualFrame::DescriptorsT desc_eigen;
        desc_eigen.conservativeResize(splited.size(),1);
        descriptor_size= splited.size();
        for(int i=0; i<splited.size(); i++){
            desc_eigen(i,0)=(unsigned char)atoi(splited[i].c_str());
        }
        desc_eigen_list.push_back(desc_eigen);
        
    }
    for(const auto& desc_eigen: desc_eigen_list){
        desc_type descriptor(const_cast<unsigned char*>(&desc_eigen(0, 0)), desc_eigen.rows(), false);
        descriptor_refs.push_back(descriptor);
    }
    
    desc_type descriptor_zero(descriptor_size);
    descriptor_zero.SetZero();
    TreeBuilderChamo builder(descriptor_zero);
    builder.kmeans().SetRestarts(FLAGS_lc_num_kmeans_restarts);
    std::cout<<(int)descriptor_refs[100][2]<<std::endl;
    builder.Build(descriptor_refs, FLAGS_lc_kmeans_splits, FLAGS_lc_kmeans_levels);
    std::cout<<"descriptor_refs: "<<descriptor_refs.size()<<std::endl;
    printf("%lu centers\n", builder.tree().centers().size());
    std::ifstream in_stream("/home/chamo/Documents/work/orb_mapping_loc/track_desc/orb_project_mat.dat", std::ios_base::binary);
    Eigen::MatrixXf projection_matrix_;
    common::Deserialize(&projection_matrix_, &in_stream);
    in_stream.close();
    const TreeBuilderChamo::Tree& tree = builder.tree();
    std::vector<desc_type> centers = tree.centers();
    Eigen::MatrixXf projected_descriptor_1;
    Eigen::MatrixXf projected_descriptor_2;
    for(int i=0; i<centers.size(); i++){
        Eigen::Map<aslam::VisualFrame::DescriptorsT> desc_eigen(centers[i].data(), descriptor_size, 1);
        Eigen::VectorXf projected_descriptor;
        descriptor_projection::ProjectDescriptor(desc_eigen, projection_matrix_, 10, projected_descriptor);
        Eigen::MatrixXf projected_descriptor_1_item=projected_descriptor.block(0,0,5,1);
        Eigen::MatrixXf projected_descriptor_2_item=projected_descriptor.block(5,0,5,1);
        append_eigen_desc(projected_descriptor_1, projected_descriptor_1_item );
        append_eigen_desc(projected_descriptor_2, projected_descriptor_2_item );
        //std::cout<<projected_descriptor_1_item.transpose()<<std::endl;
    }

    std::ofstream out_stream("/home/chamo/Documents/work/orb_mapping_loc/track_desc/words.dat", std::ios_base::binary);
    int deserialized_version=0;
    int serialized_target_dimensionality=5;
    common::Serialize(deserialized_version, &out_stream);
    common::Serialize(serialized_target_dimensionality, &out_stream);
    common::Serialize(projection_matrix_, &out_stream);
    common::Serialize(projected_descriptor_1, &out_stream);
    common::Serialize(projected_descriptor_2, &out_stream);
    out_stream.close();
    
}
