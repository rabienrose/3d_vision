#include <iostream>
#include <dirent.h>

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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "read_write_data_lib/read_write.h"
#include "orb_slam_lib/two_frame_pose.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

DEFINE_string(images_folder, "", "images ");
DEFINE_string(ncamera_calibration, "", "traj ");
DEFINE_int32(start_frame, 0, "traj ");
DEFINE_int32(end_frame, -1, "traj ");
DEFINE_string(quantizer_filename, "", "traj ");
DEFINE_string(index_addr, "", "traj ");
DEFINE_bool(generate_db, true, "traj ");

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

}  // namespace std

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

std::string type2str(int type) {
  std::string r;
  using namespace cv;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
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

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    visualization::RVizVisualizationSink::init();
    
    if (FLAGS_images_folder.empty()) {
        return -1;
    }

    int scale_rate=1;
    
    std::ifstream in_stream(FLAGS_quantizer_filename, std::ios_base::binary);
    int deserialized_version;
    common::Deserialize(&deserialized_version, &in_stream);

    int serialized_target_dimensionality;
    Eigen::MatrixXf words_first_half_;
    Eigen::MatrixXf words_second_half_;
    Eigen::MatrixXf projection_matrix_;

    common::Deserialize(&serialized_target_dimensionality, &in_stream);
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
    
    if(FLAGS_generate_db==false){
        loop_closure::proto::InvertedMultiIndex proto_inverted_multi_index;
        std::fstream input(FLAGS_index_addr.c_str(), std::ios::in | std::ios::binary);
        if (!proto_inverted_multi_index.ParseFromIstream(&input)) {
            std::cerr << "Failed to parse map data." << std::endl;
        }
        index_->deserialize(proto_inverted_multi_index);
        std::string posi_addr="/home/chamo/Documents/work/orb_mapping_loc/posi.txt";
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> mp_posis;
        CHAMO::read_mp_posi(posi_addr, mp_posis);
        Eigen::Matrix3Xd points1;
        points1.resize(3,mp_posis.size());
        for(int i=0; i<mp_posis.size(); i++){
            points1(0,i)=mp_posis[i].x();
            points1(1,i)=mp_posis[i].y();
            points1(2,i)=mp_posis[i].z();
        }    
        visualization::publish3DPointsAsPointCloud(points1, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,"chamo_target1");
        visualization::PoseVector poses_vis;
        for(int n=FLAGS_start_frame; n<FLAGS_end_frame ;n=n+1)
        {
            std::stringstream ss;
            ss<<"img_"<<n<<".jpg";
        
            std::string img_addr = FLAGS_images_folder + ss.str();
            cv::Mat desc_list;
            std::vector<cv::KeyPoint> kps_list;
            std::vector<std::vector<std::vector<std::size_t>>> mGrid;
            std::string cam_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/camera_config.txt";
            Eigen::Matrix3d cam_inter;
            Eigen::Vector4d cam_distort;
            Eigen::Matrix4d Tbc;
            CHAMO::read_cam_info(cam_addr, cam_inter, cam_distort, Tbc);
            cv::Mat cam_inter_cv;
            cv::Mat cam_distort_cv;
            convert_eigen_double_mat_float(cam_inter, cam_inter_cv);
            convert_eigen_double_mat_float(cam_distort, cam_distort_cv);
            std::vector<std::vector<std::vector<std::size_t>>> mGrid;
            orb_slam::ExtractOrb(img_addr, desc_list, kps_list, mGrid, cam_inter_cv, cam_distort_cv);
            
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
                out_indices.resize(1);
                Eigen::VectorXf out_distances;
                out_distances.resize(1);
                Eigen::Matrix<float, 10 ,1> projected_desc_fix;
                projected_desc_fix=projected_desc;
                index_->GetNNearestNeighbors(projected_desc_fix, 1, out_indices, out_distances);
                //std::cout<<out_indices.transpose()<<std::endl;
                //std::cout<<out_distances.transpose()<<std::endl;
                for(int j=0; j<out_indices.size(); j++){
                    
                    if(out_indices[j]!=-1){
                        //
                        int frame_index = index_->get_desc_frameid(out_indices[j]);
                        int track_index = index_->get_desc_trackid(out_indices[j]);
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
//             for(auto item: frame_to_match){
//                 std::cout<<item.first<<":"<<item.second.size()<<std::endl;
//                 for(int k=0; k<item.second.size(); k++){
//                     std::cout<<item.second[k].desc_id_tar<<",";
//                 }
//                 
//             }
            
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
                            return 0;
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
                    }
                    //std::cout << "["<<item.first<<": "<<item.second.size()<<"]";
                }
                Eigen::Matrix3Xd points;
                points.resize(3,point3ds.size());
                for(int i=0; i<point3ds.size(); i++){
                    points(0,i)=point3ds[i].x;
                    points(1,i)=point3ds[i].y;
                    points(2,i)=point3ds[i].z;
                }    
                visualization::publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,"chamo_target");
                //std::cout<<std::endl;
                cv::Mat rvec;
                cv::Mat tvec;
                cv::Mat inliers;
                cv::Mat cam_distort_zero;
                cv::solvePnPRansac(point3ds, point2ds, cam_inter_cv, cam_distort_zero, rvec, tvec, false, 1000, 3.0f, 0.99, inliers);
                if(inliers.rows<20){
                    continue;
                }
                std::cout<<"inliers.rows: "<<inliers.rows<<std::endl;
                cv::Mat rot_m;
                cv::Rodrigues(rvec, rot_m);
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> rot_m_eigen;
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tvec_eigen;
                convert_mat_float_eigen_double(rot_m_eigen, rot_m);
                convert_mat_float_eigen_double(tvec_eigen, tvec);
                Eigen::Matrix4d pose_inv=Eigen::Matrix4d::Identity();
                pose_inv.block(0,0,3,3)=rot_m_eigen;
                pose_inv.block(0,3,3,1)=tvec_eigen;
                Eigen::Matrix4d pose_eigen=pose_inv.inverse();
                visualization::Pose pose;
                pose.G_p_B = pose_eigen.block(0,3,3,1);
                Eigen::Matrix3d rot_eigen = pose_eigen.block(0,0,3,3);
                Eigen::Quaterniond rot_q(rot_eigen);
                pose.G_q_B = rot_q;

                pose.id =poses_vis.size();
                pose.scale = 0.2;
                pose.line_width = 0.02;
                pose.alpha = 1;
                poses_vis.push_back(pose);
                visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", "match_result");
                
            }
        }
        ros::spin();
    }else{
        std::string track_addr="/home/chamo/Documents/work/orb_mapping_loc/track.txt";
        std::vector<std::vector<int>> tracks;
        CHAMO::read_track_info(track_addr, tracks);
        
        std::string desc_addr="/home/chamo/Documents/work/orb_mapping_loc/desc.txt";
        std::vector<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>> descs;
        CHAMO::read_desc_eigen(desc_addr, descs);
        
        std::string kp_addr="/home/chamo/Documents/work/orb_mapping_loc/kps.txt";
        std::vector<Eigen::Vector2f> kp_uvs;
        std::vector<int> kp_frameids;
        std::vector<int> kp_octoves;
        CHAMO::read_kp_info(kp_addr, kp_uvs, kp_frameids, kp_octoves);
        
        std::string img_time_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/camera_1_image_time.txt";
        std::string pose_addr="/home/chamo/Documents/work/orb_mapping_loc/traj.txt";
        std::map<double, int> pose_list;
        std::map<int, int> frame_ids;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> pose_vec;
        CHAMO::read_pose_list(pose_list, frame_ids, pose_vec, pose_addr, img_time_addr);
        
        Eigen::VectorXf projected_desc;
        Eigen::VectorXf projected_desc_c;
        int desc_count=0;
        for(int i=0; i<tracks.size(); i++){
            int mp_id=i;
            for(int j=0; j<tracks[i].size(); j++){
                int kp_id=tracks[i][j];
                descriptor_projection::ProjectDescriptor(descs[kp_id], projection_matrix_, 10, projected_desc);
                index_->AddDescriptors(projected_desc, frame_ids[kp_frameids[kp_id]], mp_id);
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
        std::fstream output(FLAGS_index_addr.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
        if (!proto_inverted_multi_index.SerializeToOstream(&output)) {
            std::cerr << "Failed to write map data." << std::endl;
        }
    }
    
    return 0;
}
