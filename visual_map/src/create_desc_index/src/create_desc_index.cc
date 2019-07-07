#include "create_desc_index.h"
#include "read_write_data_lib/read_write.h"

#include <Eigen/Core>
#include <inverted-multi-index/inverted-index.h>
#include <inverted-multi-index/inverted-multi-index.h>
#include <inverted-multi-index/kd-tree-index.h>

#include <maplab-common/binary-serialization.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "inverted-multi-index/inverted_multi_index.pb.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
namespace DescIndex
{
    void findIdByName(std::vector<std::string>& names, int& re_id, std::string query_name){
        re_id=-1;
        for(int i=0; i<names.size(); i++){
            if(names[i]==query_name){
                re_id=i;
                return;
            }
        }
        return;
    }
    
    void create_desc_index(std::string resource_dir, std::string map_name){
        std::ifstream in_stream(resource_dir+"/words_projmat.dat", std::ios_base::binary);
        int deserialized_version;
        common::Deserialize(&deserialized_version, &in_stream);
        int serialized_target_dimensionality;
        Eigen::MatrixXf words_first_half_;
        Eigen::MatrixXf words_second_half_;
        Eigen::MatrixXf projection_matrix_;
        common::Deserialize(&serialized_target_dimensionality, &in_stream);
        std::cout<<serialized_target_dimensionality<<std::endl;
        common::Deserialize(&projection_matrix_, &in_stream);
        
        std::cout<<projection_matrix_.cols()<<":"<<projection_matrix_.rows()<<std::endl;
        //std::cout<<projection_matrix_<<std::endl;
        common::Deserialize(&words_first_half_, &in_stream);
        
        common::Deserialize(&words_second_half_, &in_stream);
        std::cout<<words_second_half_.cols()<<":"<<words_second_half_.rows()<<std::endl;

        const Eigen::MatrixXf& words_1 = words_first_half_;
        const Eigen::MatrixXf& words_2 = words_second_half_;
        Eigen::MatrixXf words_;
        words_.resize(words_first_half_.rows()*2, words_first_half_.cols());
        words_.block(0,0,words_first_half_.rows(), words_first_half_.cols())=words_first_half_;
        words_.block(words_first_half_.rows(),0,words_first_half_.rows(), words_first_half_.cols())=words_second_half_;
        CHECK_GT(words_1.cols(), 0);
        CHECK_GT(words_2.cols(), 0);

        std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_; 
        index_.reset(new loop_closure::inverted_multi_index::InvertedMultiIndex<5>(words_1, words_2, 10));
        //std::shared_ptr<loop_closure::inverted_index::InvertedIndex<10>> index_; 
        //index_.reset(new loop_closure::inverted_index::InvertedIndex<10>(words_, 10));
        //std::shared_ptr<loop_closure::kd_tree_index::KDTreeIndex<10>> index_;
        //index_.reset(new loop_closure::kd_tree_index::KDTreeIndex<10>());
        
        vm::VisualMap map;
        vm::loader_visual_map(map, resource_dir+"/"+map_name);
        map.ComputeUniqueId();
        
        Eigen::VectorXf projected_desc;
        Eigen::VectorXf projected_desc_c;
        int desc_count=0;
        for(int i=0; i<map.mappoints.size(); i++){
            int mp_id=i;
            for(int j=0; j<map.mappoints[i]->track.size(); j++){
                std::shared_ptr<vm::Frame> frame_p=map.mappoints[i]->track[j].frame;
                int kp_ind=map.mappoints[i]->track[j].kp_ind;
                Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> raw_descriptor;
                frame_p->getDesc(kp_ind, raw_descriptor);
                //std::cout<<raw_descriptor.tranpose()<<std::endl;
                descriptor_projection::ProjectDescriptor(raw_descriptor, projection_matrix_, 10, projected_desc);
                index_->AddDescriptors(projected_desc, frame_p->id, mp_id);
                //std::cout<<projected_desc.transpose()<<std::endl;
                if(desc_count==0){
                    projected_desc_c=projected_desc;
                }
                desc_count++;
            }
        }
        
    //         Eigen::VectorXi out_indices;
    //         out_indices.resize(15);
    //         Eigen::VectorXf out_distances;
    //         out_distances.resize(15);
    //         Eigen::Matrix<float, 10 ,1> projected_desc_fix;
    //         projected_desc_fix=projected_desc_c;
    //         index_->GetNNearestNeighbors(projected_desc_fix, 15, out_indices, out_distances);
    //         std::cout<<out_distances.transpose()<<std::endl;

        loop_closure::proto::InvertedMultiIndex proto_inverted_multi_index;
        index_->serialize(&proto_inverted_multi_index);
        std::fstream output(resource_dir+"/index.dat", std::ios::out | std::ios::trunc | std::ios::binary);
        if (!proto_inverted_multi_index.SerializeToOstream(&output)) {
            std::cerr << "Failed to write map data." << std::endl;
        }
    }
}