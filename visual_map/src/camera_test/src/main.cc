#include <iostream>
#include <dirent.h>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/feature-track.h>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/matcher/match.h>
#include "aslam/matcher/match-visualization.h"
#include <aslam/matcher/matching-engine-exclusive.h>
#include <aslam/matcher/matching-problem-frame-to-frame.h>
#include "feature-tracking/feature-track-extractor.h"
#include "feature-tracking/feature-detection-extraction.h"
#include <inverted-multi-index/inverted-multi-index.h>
#include <maplab-common/binary-serialization.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "inverted-multi-index/inverted_multi_index.pb.h"
#include "two_frame_pose.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(images_folder, "", "images ");
DEFINE_string(ncamera_calibration, "", "traj ");
DEFINE_int32(start_frame, 0, "traj ");
DEFINE_int32(end_frame, -1, "traj ");
DEFINE_string(quantizer_filename, "", "traj ");
DEFINE_string(index_addr, "", "traj ");
DEFINE_bool(generate_db, true, "traj ");
DEFINE_string(result_addr, "", "traj ");

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

void convert_kp_to_cv(Eigen::Matrix2Xd& eigen_kp, std::vector<cv::Point2f>& cv_kp){
    for(int i=0; i<eigen_kp.cols(); i++){
        cv::Point2f pt;
        pt.x=eigen_kp(0,i);
        pt.y=eigen_kp(1,i);
        cv_kp.push_back(pt);
    }
}

void FindRT(const cv::Mat& img1, const cv::Mat& img2, 
    double fx, double fy, double cx, double cy,
    double k1, double k2, double p1, double p2,
    Eigen::Matrix3d& R, Eigen::Vector3d& t){
    cv::Mat cam_m=cv::Mat::zeros(3, 3, CV_32FC1);
    cam_m.at<float>(0,0)=fx;
    cam_m.at<float>(1,1)=fy;
    cam_m.at<float>(0,2)=cx;
    cam_m.at<float>(1,2)=cy;
    cam_m.at<float>(2,2)=1;
    cv::Mat cam_dis(4, 1, CV_32FC1);
    cam_dis.at<float>(0,0)=k1;
    cam_dis.at<float>(1,0)=k2;
    cam_dis.at<float>(2,0)=p1;
    cam_dis.at<float>(3,0)=p2;
    cv::Mat img1_undistort;
    cv::undistort(img1, img1_undistort, cam_m, cam_dis);
    cv::Mat img2_undistort;
    cv::undistort(img2, img2_undistort, cam_m, cam_dis);
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
    detector->detectAndCompute(img1_undistort, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2_undistort, cv::noArray(), keypoints2, descriptors2);
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    std::sort(matches.begin(), matches.end());
    std::vector<cv::DMatch> good_matches;
    const int ptsPairs = std::min(1000, (int)(matches.size() * 0.6f));
    for( int j = 0; j < ptsPairs; j++ ){
        good_matches.push_back(matches[j]);
    }
    
    std::vector<cv::Point2f> points1(good_matches.size());
    std::vector<cv::Point2f> points2(good_matches.size());
    std::vector<int> points1_ind(good_matches.size());
    std::vector<int> points2_ind(good_matches.size());
    
//     std::vector<std::pair<int,int>> mvMatches12;
//     for(int i=0; i< good_matches.size();i++){
//         std::pair<int,int> match;
//         match.first=good_matches[i].queryIdx;
//         match.second=good_matches[i].trainIdx;
//         mvMatches12.push_back(match);
//     }
//     cv::Mat R21;
//     cv::Mat t21;
//     cv::Mat outlier_cv;
//     std::vector<bool> vbMatchesInliers;
//     orb_slam::FindRT(keypoints1, keypoints2, vbMatchesInliers, R21, t21, mvMatches12 , cam_m);
    
    for(int i=0; i< good_matches.size();i++){
        int kpInd1=good_matches[i].queryIdx;
        int kpInd2=good_matches[i].trainIdx;
        points1_ind[i]=kpInd1;
        points2_ind[i]=kpInd2;
        points1[i] =keypoints1[kpInd1].pt;
        points2[i] =keypoints2[kpInd2].pt;
    }
    cv::Mat outlier_cv;
    cv::Mat E;
    E = cv::findEssentialMat(points1, points2, cam_m, cv::RANSAC, 0.999, 5.0, outlier_cv);
    cv::Mat R21;
    cv::Mat t21;
    cv::recoverPose(E, points1, points2, cam_m, R21, t21,outlier_cv);
    
    
    std::cout<<R21<<std::endl;
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            R(i,j)=R21.at<double>(i,j);
        }
    }
    for (int i=0; i<3; i++){
        t(i,0)=t21.at<double>(i,0);
    }
}

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    std::cout<<FLAGS_images_folder<<std::endl;
    
    if (FLAGS_images_folder.empty()) {
        return -1;
    }

    int scale_rate=1;
    
    aslam::NCamera::Ptr camera_system = aslam::NCamera::loadFromYaml(FLAGS_ncamera_calibration);
    CHECK(camera_system) << "Could not load the camera calibration from: \'"<< FLAGS_ncamera_calibration << "\'";
    
    aslam::Camera::Ptr pinhole_A = camera_system->getCameraShared(0);
    
    std::ifstream in_stream(FLAGS_quantizer_filename, std::ios_base::binary);
    int deserialized_version;
    common::Deserialize(&deserialized_version, &in_stream);

    int serialized_target_dimensionality;
    Eigen::MatrixXf words_first_half_;
    Eigen::MatrixXf words_second_half_;
    Eigen::MatrixXf projection_matrix_;
    common::Deserialize(&serialized_target_dimensionality, &in_stream);
    common::Deserialize(&projection_matrix_, &in_stream);
    common::Deserialize(&words_first_half_, &in_stream);
    common::Deserialize(&words_second_half_, &in_stream);

    const Eigen::MatrixXf& words_1 = words_first_half_;
    const Eigen::MatrixXf& words_2 = words_second_half_;
    CHECK_GT(words_1.cols(), 0);
    CHECK_GT(words_2.cols(), 0);

    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_; 
    index_.reset(new loop_closure::inverted_multi_index::InvertedMultiIndex<5>(words_1, words_2, 10));

    
    std::unordered_map<aslam::FrameId, int> FrameId_to_index;
    float time_stamp=0;
    std::unique_ptr<feature_tracking::FeatureDetectorExtractor> tracker;
    tracker.reset(new feature_tracking::FeatureDetectorExtractor(camera_system->getCamera(0)));
    std::vector<std::shared_ptr<aslam::VisualNFrame>> frame_list;
    int count_input=0;
    for(int i=FLAGS_start_frame; i<FLAGS_end_frame ;i=i+1)
    {
        std::stringstream ss;
        ss<<"img_"<<i<<".jpg";
    
        std::string img_addr = FLAGS_images_folder + ss.str();
        cv::Mat img = cv::imread(img_addr);
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        cv::resize(img,img,cv::Size(img.cols/scale_rate, img.rows/scale_rate));
        
        aslam::VisualNFrame::Ptr nframe= aslam::VisualNFrame::createEmptyTestVisualNFrame(camera_system, time_stamp*1000*1000*1000);
        aslam::VisualFrame::Ptr frame = nframe->getFrameShared(0);
        FrameId_to_index[frame->getId()]=i;
        frame->setRawImage(img.clone());
        tracker->detectAndExtractFeatures(&(*frame));
        frame_list.push_back(nframe);
        count_input++;
    }
    
    if(FLAGS_generate_db==false){
        loop_closure::proto::InvertedMultiIndex proto_inverted_multi_index;
        std::fstream input(FLAGS_index_addr.c_str(), std::ios::in | std::ios::binary);
        if (!proto_inverted_multi_index.ParseFromIstream(&input)) {
            std::cerr << "Failed to parse map data." << std::endl;
        }
        index_->deserialize(proto_inverted_multi_index);
        std::ofstream out(FLAGS_result_addr);
        std::map<int, int> fine_match_count;
        for(int kk=0; kk<frame_list.size(); kk++){
            aslam::VisualFrame::Ptr frame_=frame_list[kk]->getFrameShared(0);
            const aslam::VisualFrame::DescriptorsT& desc = frame_->getDescriptors();
            std::map<int, int> match_count_per_frame;
            std::unordered_map<int, std::vector<Match>> frame_to_match;
            for(int i=0; i<desc.cols(); i++){
                Eigen::VectorXf projected_desc;
                descriptor_projection::ProjectDescriptor(desc.col(i), projection_matrix_, 10, projected_desc);
                Eigen::VectorXi out_indices;
                out_indices.resize(15);
                Eigen::VectorXf out_distances;
                out_distances.resize(15);
                Eigen::Matrix<float, 10 ,1> projected_desc_fix;
                projected_desc_fix=projected_desc;
                index_->GetNNearestNeighbors(projected_desc_fix, 25, out_indices, out_distances);
                
                for(int j=0; j<out_indices.size(); j++){
                    
                    if(out_indices[j]!=-1){
                        //std::cout<<out_indices[j]<<std::endl;
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

            
            out << FrameId_to_index[frame_->getId()]<<" detail: "<<std::endl;
            out << "raw match count:"<<std::endl;
            for (auto item :match_count_per_frame){
                
                out << "["<<item.first<<": "<<item.second<<"]";
            }
            out <<std::endl;
            
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
            if (max_component_size > 10) {
                const std::unordered_set<Match>& matches_max_component = components[max_component_id];
                typedef std::pair<int, int> KeypointLandmarkPair;
                std::unordered_set<KeypointLandmarkPair> used_matches;
                used_matches.reserve(2u * matches_max_component.size());
                for (const Match& structure_match : matches_max_component) {
                    const bool is_match_unique = used_matches.emplace(
                            structure_match.desc_id_tar,
                            structure_match.track_id).second;
                    if (!is_match_unique) {
                    // Skip duplicate (keypoint to landmark) structure matches.
                        continue;
                    }
                    frame_matches[structure_match.frame_id_tar].push_back(structure_match);
                    total_match_count=total_match_count+1;
                }
            }
            out <<"filtered match count:"<<std::endl;
            for (auto item :frame_matches){
                
                out << "["<<item.first<<": "<<item.second.size()<<"]";
            }
            out <<std::endl;
            fine_match_count[FrameId_to_index[frame_->getId()]]=total_match_count;
        }
        
        out << "filtered match count:"<<std::endl;
        for (auto item :fine_match_count){  
            out << "["<<item.first<<": "<<item.second<<"]";
        }
        out.close();
    }else{
        aslam::MatchingEngineExclusive<aslam::MatchingProblemFrameToFrame> matching_engine_;
        aslam::VisualFrame::Ptr apple_frame_;
        aslam::VisualFrame::Ptr banana_frame_;
        double image_space_distance_threshold_=40.0;
        int hamming_distance_threshold_=60;
        
        int track_id=0;
        aslam::PinholeCamera* pinhole = static_cast<aslam::PinholeCamera*>(pinhole_A.get());
        Eigen::Matrix3d cam_m = pinhole->getCameraMatrix();
        
        aslam::RadTanDistortion* distortion = static_cast<aslam::RadTanDistortion *>(pinhole_A->getDistortionMutable());
        const Eigen::VectorXd& dist_coef = distortion->getParameters();
        int next_trackid=0;
        for (int i=1;i<frame_list.size();i++){
            
            apple_frame_=frame_list[i-1]->getFrameShared(0);
            banana_frame_=frame_list[i]->getFrameShared(0);
            
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            FindRT(apple_frame_->getRawImage(), banana_frame_->getRawImage(), 
                cam_m(0,0), cam_m(1,1), cam_m(0,2), cam_m(1,2),
                dist_coef(0,0), dist_coef(1,0), dist_coef(2,0), dist_coef(3,0), 
                R, t);

            Eigen::Quaterniond r_qua(R.transpose());
            aslam::Quaternion q_B_A(r_qua);
            aslam::MatchingProblemFrameToFrame::Ptr matching_problem =
                aligned_shared<aslam::MatchingProblemFrameToFrame>(
                *apple_frame_, *banana_frame_, q_B_A, image_space_distance_threshold_,
                hamming_distance_threshold_);
            aslam::MatchingProblemFrameToFrame::MatchesWithScore matches_A_B;
            matching_engine_.match(matching_problem.get(), &matches_A_B);
            std::cout<<"match kf: "<<FrameId_to_index[banana_frame_->getId()]<<"|"<<matches_A_B.size()<<std::endl;
            for (const aslam::FrameToFrameMatchWithScore& match : matches_A_B) {
                int index_apple = match.getKeypointIndexAppleFrame();
                int index_banana = match.getKeypointIndexBananaFrame();
                int track_id = apple_frame_->getTrackIds()(index_apple,0);
                Eigen::VectorXi& apple_trackid=*(apple_frame_->getTrackIdsMutable());
                Eigen::VectorXi& banana_trackid=*(banana_frame_->getTrackIdsMutable());
                if(track_id==-1){
                    apple_trackid[index_apple]=next_trackid;
                    banana_trackid[index_banana]=next_trackid;
                    next_trackid++;
                }else{
                    banana_trackid[index_banana]=track_id;
                }
            }
        }
        const size_t kMaxTrackLength = 100;
        const size_t kMinTrackLength = 3;
        vio_common::FeatureTrackExtractor extractor( camera_system, kMaxTrackLength, kMinTrackLength);
        aslam::FeatureTracks all_tracks;
        for (int i = 0; i < frame_list.size(); ++i) {
            aslam::FeatureTracksList tracks;
            size_t num_tracks =extractor.extractFromNFrameStream(frame_list[i], &tracks);
            all_tracks.insert(all_tracks.end(),tracks[0].begin(), tracks[0].end());
        }
        Eigen::VectorXf projected_desc;
        int desc_count=0;
        for(int i=0; i<all_tracks.size(); i++){
            const aslam::KeypointIdentifierList& kps = all_tracks[i].getKeypointIdentifiers();
            for(int j=0; j<kps.size(); j++){
                const aslam::VisualFrame& frame= kps[j].getFrame();
                const aslam::VisualFrame::DescriptorsT& desc = frame.getDescriptors();
                
                descriptor_projection::ProjectDescriptor(desc.col(kps[j].getKeypointIndex()), projection_matrix_, 10, projected_desc);
                if(FrameId_to_index.count(frame.getId())==0){
                    //std::cout<<"frame id error"<<std::endl;
                }
                index_->AddDescriptors(projected_desc, FrameId_to_index[frame.getId()], i);
                desc_count++;
            }
        }
        loop_closure::proto::InvertedMultiIndex proto_inverted_multi_index;
        index_->serialize(&proto_inverted_multi_index);
        
        std::fstream output(FLAGS_index_addr.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
        if (!proto_inverted_multi_index.SerializeToOstream(&output)) {
            std::cerr << "Failed to write map data." << std::endl;
        }
    }
    return 0;
}
