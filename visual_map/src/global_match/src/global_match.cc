#include <global_match/global_match.h>
#ifndef __APPLE__
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#endif

#include "Frame.h"
struct Match{
        int query_desc_id;
        int frame_id_tar;
        int desc_id_tar;
        int track_id;
        bool operator==(const Match& other) const {
            bool result = true;
            result &= query_desc_id == other.query_desc_id;
            result &= frame_id_tar == other.frame_id_tar;
            result &= desc_id_tar == other.desc_id_tar;
            result &= track_id == other.track_id;
            return result;
        }
};

namespace std {
    template <>
    struct hash<Match> {
        std::size_t operator()( const Match& value) const {
            const std::size_t h0(std::hash<int>()(value.query_desc_id));
            const std::size_t h1(std::hash<int>()(value.frame_id_tar));
            const std::size_t h2(std::hash<int>()(value.desc_id_tar));
            const std::size_t h3(std::hash<int>()(value.track_id));
            return h0 ^ h1 ^ h2 ^ h3;
        }
    };

    template <>
    struct hash<std::pair<int, int>> {
        std::size_t operator()(const std::pair<int, int>& value)
            const {
                const std::size_t h1(std::hash<int>()(value.first));
                const std::size_t h2(std::hash<int>()(value.second));
            return h1 ^ h2;
        }
    };
}


namespace chamo {

    
    void convert_mat_eigen(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& matrix, cv::Mat mat){
        //Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> m_temp;
        matrix.resize(mat.rows, mat.cols);
        for(int i=0; i<mat.rows; i++){
            for(int j=0; j<mat.cols; j++){
                //std::cout<<(int)mat.at<unsigned char>(i, j)<<",";
                matrix(i, j) = mat.at<unsigned char>(i, j);
            }
        }
        matrix.transposeInPlace();
    }
    
    void convert_mat_float_eigen_double(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix, cv::Mat mat){
        matrix.resize(mat.rows, mat.cols);
        for(int i=0; i<mat.rows; i++){
            for(int j=0; j<mat.cols; j++){
                matrix(i, j) = mat.at<double>(i, j);
            }
        }
    }

    void convert_eigen_double_mat_float(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix, cv::Mat& mat){
        mat = cv::Mat(matrix.rows(), matrix.cols(), CV_32FC1);
        for(int i=0; i<matrix.rows(); i++){
            for(int j=0; j<matrix.cols(); j++){
                mat.at<float>(i, j)=matrix(i, j);
            }
        }
    }
    
    void LoadMap(std::string res_root, std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>>& index_,
        Eigen::MatrixXf& projection_matrix_, std::string index_filename){
        std::ifstream in_stream(res_root+"/words_projmat.dat", std::ios_base::binary);
        int deserialized_version;
        common::Deserialize(&deserialized_version, &in_stream);
        int serialized_target_dimensionality;
        Eigen::MatrixXf words_first_half_;
        Eigen::MatrixXf words_second_half_;
        common::Deserialize(&serialized_target_dimensionality, &in_stream);
        common::Deserialize(&projection_matrix_, &in_stream);
        common::Deserialize(&words_first_half_, &in_stream);
        common::Deserialize(&words_second_half_, &in_stream);

        index_.reset(new loop_closure::inverted_multi_index::InvertedMultiIndex<5>(words_first_half_, words_second_half_, 10));
        loop_closure::proto::InvertedMultiIndex proto_inverted_multi_index;
        std::fstream input(res_root+"/"+index_filename, std::ios::in | std::ios::binary);
        if (!proto_inverted_multi_index.ParseFromIstream(&input)) {
            std::cerr << "Failed to parse map data." << std::endl;
        }
        index_->deserialize(proto_inverted_multi_index);
    }

    
    void MatchImg(std::vector<Eigen::Vector3d>& mp_posis, std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>>& index_,
        Eigen::MatrixXf& projection_matrix_, ORB_SLAM2::Frame& frame, std::vector<int>& inliers_mp, std::vector<int>& inliers_kp,
        Eigen::Matrix4d& pose
    ){
        cv::Mat desc_list=frame.mDescriptors;
        std::vector<cv::KeyPoint>& kps_list=frame.mvKeysUn;
            
        std::map<int, int> match_count_per_frame;
        std::unordered_map<int, std::vector<Match>> frame_to_match;
        for(int i=0; i<desc_list.rows; i++){
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> desc_eigen;
            convert_mat_eigen(desc_eigen, desc_list.row(i));
            Eigen::VectorXf projected_desc;
            descriptor_projection::ProjectDescriptor(desc_eigen, projection_matrix_, 10, projected_desc);
            //std::cout<<desc_list.row(i)<<std::endl;
            //std::cout<<desc_eigen.transpose().cast<int>()<<std::endl;
            //std::cout<<projected_desc.transpose()<<std::endl;
            Eigen::VectorXi out_indices;
            int condi_count=3;
            out_indices.resize(condi_count);
            Eigen::VectorXf out_distances;
            out_distances.resize(condi_count);
            Eigen::Matrix<float, 10 ,1> projected_desc_fix;
            projected_desc_fix=projected_desc;
            index_->GetNNearestNeighbors(projected_desc_fix, condi_count, out_indices, out_distances);
            //std::cout<<out_indices.transpose()<<std::endl;
            //std::cout<<out_distances.transpose()<<std::endl;
            for(int j=0; j<out_indices.size(); j++){
                
                if(out_indices[j]!=-1){
                    if(out_distances[j]>3){
                        continue;
                    }
                    int frame_index = index_->get_desc_frameid(out_indices[j]);
                    int track_index = index_->get_desc_trackid(out_indices[j]);
                    //std::cout<<frame_index<<":"<<track_index<<std::endl;
                    Match match;
                    match.desc_id_tar=out_indices[j];
                    match.frame_id_tar=frame_index;
                    match.query_desc_id=i;
                    match.track_id=track_index;
                    if(frame_to_match.find(match.frame_id_tar)==frame_to_match.end()){
                        std::vector<Match> matches;
                        matches.push_back(match);
                        frame_to_match[match.frame_id_tar]=matches;
                    }else{
                        frame_to_match[match.frame_id_tar].push_back(match);
                    }

                    if(match_count_per_frame.count(frame_index)!=0){
                        match_count_per_frame[frame_index]++;
                    }else{
                        match_count_per_frame[frame_index]=1;
                    }
                }
            }
            
        }
        
        typedef int ComponentId;
        constexpr ComponentId kInvalidComponentId = -1;
        typedef std::unordered_map<Match, ComponentId> MatchesToComponents;
        typedef std::unordered_map<ComponentId, std::unordered_set<Match>> Components;
        typedef std::unordered_map<int, std::vector<Match>> LandmarkMatches;

        MatchesToComponents matches_to_components;
        LandmarkMatches landmark_matches;

        for (const std::pair<int, std::vector<Match>>& id_matches_pair : frame_to_match) {
            for (const Match& match : id_matches_pair.second) {
                landmark_matches[match.track_id].emplace_back(match);
                matches_to_components.emplace(match, kInvalidComponentId);
            }
        }

        ComponentId count_component_index = 0;
        size_t max_component_size = 0u;
        ComponentId max_component_id = kInvalidComponentId;
        Components components;
        for (const MatchesToComponents::value_type& match_to_component : matches_to_components) {
            if (match_to_component.second != kInvalidComponentId)
                continue;
            ComponentId component_id = count_component_index++;

            // Find the largest set of keyframes connected by landmark covisibility.
            std::queue<Match> exploration_queue;
            exploration_queue.push(match_to_component.first);
            while (!exploration_queue.empty()) {
                const Match& exploration_match = exploration_queue.front();

                if (match_count_per_frame[exploration_match.frame_id_tar]<5) {
                    exploration_queue.pop();
                    continue;
                }

                const MatchesToComponents::iterator exploration_match_and_component = matches_to_components.find(exploration_match);
                CHECK(exploration_match_and_component != matches_to_components.end());

                if (exploration_match_and_component->second == kInvalidComponentId) {
                    // Not part of a connected component.
                    exploration_match_and_component->second = component_id;
                    components[component_id].insert(exploration_match);
                    // Mark all observations (which are matches) from this ID (keyframe or
                    // vertex) as visited.
                    if(frame_to_match.count(exploration_match.frame_id_tar)==0){
                        return;
                    }
                    const std::vector<Match>& id_matches = frame_to_match[exploration_match.frame_id_tar];
                    for (const Match& id_match : id_matches) {
                        matches_to_components[id_match] = component_id;
                        components[component_id].insert(id_match);

                        // Put all observations of this landmark on the stack.
                        const std::vector<Match>& lm_matches =
                            landmark_matches[id_match.track_id];
                        for (const Match& lm_match : lm_matches) {
                            if (matches_to_components[lm_match] == kInvalidComponentId) {
                                exploration_queue.push(lm_match);
                            }
                        }
                    }

                    if (components[component_id].size() > max_component_size) {
                        max_component_size = components[component_id].size();
                        max_component_id = component_id;
                    }
                }
                exploration_queue.pop();
            }
        }
        
        std::map<int, std::vector<Match>> frame_matches;
        // Only store the structure matches if there is a relevant amount of them.
        int total_match_count=0;
        
        //std::cout<<"max_component_size: "<<max_component_size<<std::endl;
        std::vector<int> ransac_to_kpid;
        std::vector<int> ransac_to_mpid;
        if (max_component_size > 10) {
            
            const std::unordered_set<Match>& matches_max_component = components[max_component_id];
            typedef std::pair<int, int> KeypointLandmarkPair;
            std::unordered_set<KeypointLandmarkPair> used_matches;
            used_matches.reserve(2u * matches_max_component.size());
            for (const Match& structure_match : matches_max_component) {
                const bool is_match_unique = used_matches.emplace(structure_match.desc_id_tar, structure_match.track_id).second;
                //std::cout<<structure_match.desc_id_tar<<":"<<structure_match.track_id<<std::endl;
                if (!is_match_unique) {
                // Skip duplicate (keypoint to landmark) structure matches.
                    continue;
                }
                frame_matches[structure_match.frame_id_tar].push_back(structure_match);
                total_match_count=total_match_count+1;
            }

            std::vector<cv::Point3f> point3ds;
            std::vector<cv::Point2f> point2ds;
            for (auto item :frame_matches){
                for(int i=0; i<item.second.size(); i++){
                    cv::Point2f pt=kps_list[item.second[i].query_desc_id].pt;
                    point2ds.push_back(pt);
                    cv::Point3f posi;
                    posi.x= mp_posis[item.second[i].track_id].x();
                    posi.y= mp_posis[item.second[i].track_id].y();
                    posi.z= mp_posis[item.second[i].track_id].z();
                    point3ds.push_back(posi);
                    ransac_to_kpid.push_back(item.second[i].query_desc_id);
                    ransac_to_mpid.push_back(item.second[i].track_id);
                }
                //std::cout << "["<<item.first<<": "<<item.second.size()<<"]";
            }
            //std::cout<<std::endl;
            cv::Mat rvec;
            cv::Mat tvec;
            cv::Mat inliers;
            cv::Mat cam_distort_zero=frame.mDistCoef.clone();
            cam_distort_zero.at<float>(0)=0;
            cam_distort_zero.at<float>(1)=0;
            cam_distort_zero.at<float>(2)=0;
            cam_distort_zero.at<float>(3)=0;
            cv::Mat cam_inter_cv = frame.mK;
            //std::cout<<"point3ds: "<<point3ds.size()<<std::endl;
            cv::solvePnPRansac(point3ds, point2ds, cam_inter_cv, cam_distort_zero, rvec, tvec, false, 1000, 2.0f, 0.99, inliers, cv::SOLVEPNP_EPNP);
            
            if(inliers.rows<20){
                return;
            }
            //std::cout<<"inliers.rows: "<<inliers.rows<<std::endl;
            for(int i=0; i<inliers.rows; i++){
                //std::cout<<inliers.at<int>(i)<<std::endl;
                inliers_kp.push_back(ransac_to_kpid[inliers.at<int>(i)]);
                inliers_mp.push_back(ransac_to_mpid[inliers.at<int>(i)]);
            }
            
            cv::Mat rot_m;
            cv::Rodrigues(rvec, rot_m);
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> rot_m_eigen;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tvec_eigen;
            convert_mat_float_eigen_double(rot_m_eigen, rot_m);
            convert_mat_float_eigen_double(tvec_eigen, tvec);
            Eigen::Matrix4d pose_inv=Eigen::Matrix4d::Identity();
            pose_inv.block(0,0,3,3)=rot_m_eigen;
            pose_inv.block(0,3,3,1)=tvec_eigen;
            pose=pose_inv.inverse();
        }
        return;
    }
}
