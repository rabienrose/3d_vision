#include "sophus/sim3.hpp"
#include "vis_loc.h"
#include <algorithm>
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"

namespace visual_loc {
VisualLocalization::VisualLocalization() {}
int get_frame_id(std::string& img_name)
{
    int pos1 = img_name.find("_");
    int pos2 = img_name.find(".jpg");
    string temp = img_name.substr(pos1 + 1, pos2 - pos1 - 1);
    int frame_id = std::stoi(temp);

    return frame_id;
}
bool VisualLocalization::SetInit(const std::string& work_dir)
{
    workspace = work_dir;
    std::string strSettingsFile = workspace + "/vslam.yaml";
    std::string strVocFile = workspace + "/FreakAll.bin";
    mpsys = new ORB_SLAM2::System(false);
    align_init = false;
    track_init = false;
    former_match = false;
    no_align_count = MAX_NO_ALINED_COUNT;
    for (int i = 0; i < 3; i++) {
        check_align_history[i] = false;
    }
}

bool VisualLocalization::ResetInit()
{
    delete mpsys;
    std::string strSettingsFile = workspace + "/vslam.yaml";
    std::string strVocFile = workspace + "/FreakAll.bin";
    mpsys = new ORB_SLAM2::System(false);
    align_init = false;
    track_init = false;
    former_match = false;
    no_align_count = MAX_NO_ALINED_COUNT;
    for (int i = 0; i < 3; i++) {
        check_align_history[i] = false;
    }
}

bool VisualLocalization::AddImage(cv::Mat& img,
                                  std::string& img_name,
                                  double& time_stamp,
                                  Eigen::Vector3d& output_posi)
{
    // get local frame pose
    clock_t start,finish;
    double totaltime;
    start=clock();
    cv::Mat Tcw = mpsys->TrackMonocular(img, time_stamp, img_name);
    Eigen::Matrix4d frame_pose_eigen = Eigen::Matrix4d::Identity();
    bool is_keyframe = false;
    ORB_SLAM2::Map* mpmap;
    std::vector<ORB_SLAM2::KeyFrame*> vpKFs;
    std::vector<ORB_SLAM2::MapPoint*> mps_all;
    std::vector<ORB_SLAM2::MapPoint*> mps_local;
    // LOG(INFO)<<"Tracking DONE";
    if (Tcw.cols > 0) {
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
        Eigen::Matrix3d rot;
        Eigen::Vector3d trans;
        cv::cv2eigen(Rwc, rot);
        cv::cv2eigen(twc, trans);
        frame_pose_eigen.block(0, 0, 3, 3) = rot;
        frame_pose_eigen.block(0, 3, 3, 1) = trans;
        track_init = true;
        mpmap = mpsys->getMapPointer();
        vpKFs = mpmap->GetAllKeyFrames();
        mps_all = mpmap->GetAllMapPoints();
        is_keyframe = mpsys->getTrackPointer()->created_new_kf;
        mps_local = mpsys->getTrackPointer()->GetmvpLocalMapPoints();
        // LOG(INFO)<<"LOCAL MP size: "<<mpsys->getTrackPointer()->GetmvpLocalMapPoints().size();
    }
    // Tracking lost after initilization, reset tracker
    else if (track_init) {
        ResetInit();
        track_init = false;
    }
    finish=clock();
    totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
    std::cout<<"Track Time: "<<totaltime<<std::endl;

    start=clock();
    // get global pose
    ORB_SLAM2::Frame frame = mpsys->getCurrentFrame();
    float reproject_err_t;
    int match_count_t;
    int mp_count_t;
    int kf_count_t;
    cv::Mat img_display;
    mpsys->getDebugImg(img_display, reproject_err_t, match_count_t, mp_count_t, kf_count_t);
    if (!img_display.empty()) {
        cv::imshow("chamo", img_display);
        cv::waitKey(5);
    }
    // LOG(INFO)<<"Global Matching DONE";
    std::vector<int> inliers_mp;
    std::vector<int> inliers_kp;
    Eigen::Matrix4d pose;
    chamo::MatchImg(mp_posis, index_, projection_matrix_, frame, inliers_mp, inliers_kp, pose);

    finish=clock();
    totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
    std::cout<<"Global Match Time: "<<totaltime<<std::endl;

    // align the map
    bool get_global_match = inliers_kp.size() > 0;
    int frame_id = get_frame_id(img_name);
    bool is_align = false;
    std::map<int, Eigen::Vector3d> local_kp_global_mp;
    // LOG(INFO) << "get_global_match: " << get_global_match << " Tcw: " << Tcw.cols;

    if (get_global_match) {
        global_pose.insert(std::pair<int, Eigen::Matrix4d>(frame_id, pose));
        // construct global mp-- kp pairs
        for (int i = 0; i < inliers_kp.size(); ++i) {
            if (frame.mvpMapPoints[inliers_kp[i]]) {
                global_id_local_pointer[inliers_mp[i]] = frame.mvpMapPoints[inliers_kp[i]];
                // global_id_local_pointer.insert(
                // std::pair<int, ORB_SLAM2::MapPoint*>(inliers_mp[i],
                // frame.mvpMapPoints[inliers_kp[i]]));
            }
            local_kp_global_mp.insert(
                    std::pair<int, Eigen::Vector3d>(inliers_kp[i], mp_posis[inliers_mp[i]]));
        }
    }

    UpdateGlobalLocalMatches(mps_local);
    // LOG(INFO)<<"UpdateGlobalLocalMatches DONE";

    start=clock();
    bool b_force_align = get_global_match && !former_match;
    // bool b_enough_size = vpKFs.size() >= 10 && global_pose.size() >= 10 && get_global_match;
    bool b_enough_size = get_global_match;
    if ((is_keyframe || b_force_align) && b_enough_size) {
        if (frame.mpReferenceKF == NULL)
            return false;
        // std::vector<ORB_SLAM2::KeyFrame *> covisible_kf =
        // frame.mpReferenceKF->GetBestCovisibilityKeyFrames(40);
        // is_align = AlignLocalMap(frame_pose_eigen, vpKFs, covisible_kf, mps_all);
        is_align = AlignLocalMapByMapPoimt(frame_pose_eigen,
                                           vpKFs,
                                           mps_all,
                                           frame,
                                           local_kp_global_mp);
    }
    finish=clock();
    totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
    std::cout<<"Align Time: "<<totaltime<<std::endl;

    // LOG(INFO)<<"Align Map DONE";
    if (is_align || (Tcw.cols > 0 && no_align_count < MAX_NO_ALINED_COUNT)) {
        Eigen::Vector3d local_posi = frame_pose_eigen.block(0, 3, 3, 1);
        output_posi = local_posi;
        if (is_align) {
            no_align_count = 0;
            former_match = true;
        }
        if (!is_align && is_keyframe) {
            no_align_count++;
            former_match = false;
        }
        // LOG(INFO) << "LOCAL";
        return true;
    } else if (get_global_match > 0) {
        Eigen::Vector3d global_posi = pose.block(0, 3, 3, 1);
        output_posi = global_posi;
        no_align_count = MAX_NO_ALINED_COUNT;
        former_match = false;
        // LOG(INFO) << "GLOBAL";
        return true;
    }

    LOG(INFO) << "Fail To Localize";
    no_align_count = MAX_NO_ALINED_COUNT;
    former_match = false;
    return false;
}

void VisualLocalization::UpdateGlobalLocalMatches(std::vector<ORB_SLAM2::MapPoint*> mps_local)
{
    for (auto it = global_id_local_pointer.begin(); it != global_id_local_pointer.end();) {
        long unsigned int local_mp_id = it->second->mnId;
        bool in_local = false;
        for (const auto& mp_local : mps_local) {
            if (local_mp_id == mp_local->mnId) {
                in_local = true;
                break;
            }
        }

        if (!in_local) {
            global_id_local_pointer.erase(it++);
            continue;
        }
        it++;
    }
}

void VisualLocalization::LoadMapLabMap()
{

    chamo::LoadMap(workspace, index_, projection_matrix_);
    vm::VisualMap map;
    vm::loader_visual_map(map, workspace + "/opti_chamo_1000.map");
    map.GetMPPosiList(mp_posis);
}

Eigen::Vector3d VisualLocalization::GetGlobalPosition(string& img_name)
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    if (global_pose.count(get_frame_id(img_name))) {
        position = global_pose[get_frame_id(img_name)].block(0, 3, 3, 1);
    }
    return position;
}

Eigen::Vector3d VisualLocalization::GetLocalPosition()
{
    Eigen::Vector3d position = local_pose.back();
    return position;
}
bool VisualLocalization::AlignLocalMap(Eigen::Matrix4d& frame_pose,
                                       std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                                       std::vector<ORB_SLAM2::KeyFrame*>& covisible_kf,
                                       std::vector<ORB_SLAM2::MapPoint*>& mps_all)
{
    local_posis.clear();
    global_posis.clear();
    // local_poses.clear();
    // global_poses.clear();
    local_posis.reserve(covisible_kf.size());
    global_posis.reserve(covisible_kf.size());
    // local_poses.resize(vpKFs.size());
    // global_poses.resize(vpKFs.size());
    int count = 0;
    for (int i = 0; i < covisible_kf.size(); i++) {
        ORB_SLAM2::KeyFrame* pKF = covisible_kf[i];
        // while (pKF->isBad()) {
        // continue;
        // }
        cv::Mat Trw = pKF->GetPose();
        cv::Mat Rwc = Trw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Trw.rowRange(0, 3).col(3);
        Eigen::Vector3d posi(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
        auto iter = global_pose.find(get_frame_id(pKF->file_name_));
        if (iter == global_pose.end()) {
            continue;
        }
        Eigen::Vector3d tmp = iter->second.block(0, 3, 3, 1);
        global_posis.push_back(tmp);
        // global_poses[count] = iter->second;
        local_posis.push_back(posi);
        count++;
    }

    // LOG(INFO) << "global size final: " << global_posis.size();
    if (global_posis.size() != local_posis.size())
        return false;
    if (global_posis.size() < 10)
        return false;

    double scale_12;
    Eigen::Matrix4d T12;
    orb_slam::ComputeSim3(global_posis, local_posis, T12, scale_12);

    bool check_align = CheckAlign(global_posis, local_posis, T12, scale_12, 2.0, true);
    if (!check_align) {
        return false;
    }

    mp_before.clear();
    mp_after.clear();

    // update local map and pose of current frame
    UpdateLocalMap(T12, scale_12, frame_pose, vpKFs, mps_all);
    return true;
}

bool VisualLocalization::AlignLocalMapByMapPoimt(Eigen::Matrix4d& frame_pose,
                                                 std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                                                 std::vector<ORB_SLAM2::MapPoint*>& mps_all,
                                                 ORB_SLAM2::Frame& frame,
                                                 std::map<int, Eigen::Vector3d>& local_kp_global_mp)
{
    // construct mappoint matches
    std::vector<Eigen::Vector3d> global_mp;
    std::vector<Eigen::Vector3d> local_mp;

    global_mp.reserve(local_kp_global_mp.size());
    local_mp.reserve(local_kp_global_mp.size());

    for (const auto& id_mp : global_id_local_pointer) {
        cv::Mat local_mp_cv = id_mp.second->GetWorldPos();
        Eigen::Vector3d local_mp_eigen;
        cv::cv2eigen(local_mp_cv, local_mp_eigen);
        global_mp.push_back(mp_posis[id_mp.first]);
        local_mp.push_back(std::move(local_mp_eigen));
    }
    // for (const auto& kp_mp : local_kp_global_mp) {
    // if (frame.mvpMapPoints[kp_mp.first]) {
    // cv::Mat local_mp_cv = frame.mvpMapPoints[kp_mp.first]->GetWorldPos();
    // Eigen::Vector3d local_mp_eigen;
    // cv::cv2eigen(local_mp_cv, local_mp_eigen);
    // global_mp.push_back(kp_mp.second);
    // local_mp.push_back(std::move(local_mp_eigen));
    // }
    // }

    // LOG(INFO) << "global_mp: " << global_mp.size();
    if (global_mp.size() < 20) {
        return false;
    }

    // compute sim3
    double scale_12;
    Eigen::Matrix4d T12;
    // orb_slam::ComputeSim3(global_mp, local_mp, T12, scale_12);

    bool check_align1 = ComputeSim3Ransac(global_mp, local_mp, T12, scale_12);
    // LOG(INFO) << "ComputeSim3Ransac Done";
    // bool check_align = CheckAlign(global_mp, local_mp, T12, scale_12, 15.0, false);
    if (!(check_align1)) {
        return false;
    }

    LOG(INFO) << "Align!!!";
    // update local map and pose of current frame
    // LOG(INFO)<<"T12: "<<T12<<"  scale: "<<scale_12;
    clock_t start,finish;
    double totaltime;
    start = clock();
    UpdateLocalMap(T12, scale_12, frame_pose, vpKFs, mps_all);
    finish=clock();
    totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
    std::cout<<"Update Time: "<<totaltime<<std::endl;

    return true;
}


void VisualLocalization::UpdateLocalMap(const Eigen::Matrix4d& T12,
                                        const double& scale,
                                        Eigen::Matrix4d& frame_pose,
                                        std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                                        std::vector<ORB_SLAM2::MapPoint*>& mps_all)
{
    Eigen::Matrix4d T12_update = T12;
    double scale_update = scale;
    Sophus::Sim3d T12_sim3(T12);
    Sophus::Sim3d T12_sim3_update;
    Eigen::Matrix4d T_local = Eigen::Matrix4d::Identity();
    Sophus::Sim3d T_local_sim3(T_local);
    //filter
    if(align_init){
        typedef Eigen::Matrix<double,7,1> Vector7d;
        Vector7d T12_lie_algibra = T12_sim3.log();
        Vector7d T_local_lie_algibra = T_local_sim3.log();
        double tmp = no_align_count / 25.0;
        double partial_update = (tmp + 0.2) / (tmp + 1.2);
        double partial_predict = 1.0 / (tmp + 1.2);
        Vector7d T_update =
                partial_update * T12_lie_algibra + partial_predict * T_local_lie_algibra;
        T12_sim3_update = Sophus::Sim3d::exp(T_update);
        T12_update = T12_sim3_update.matrix();
        scale_update = T12_sim3_update.scale();
    }
    // update MapPoint
    for (int i = 0; i < mps_all.size(); i++) {
        if (mps_all[i]->isBad())
            continue;
        cv::Mat center = mps_all[i]->GetWorldPos();
        Eigen::Vector3d center_eigen;
        cv::cv2eigen(center, center_eigen);
        // mp_before.push_back(center_eigen);
        TransformPositionUseSim3(T12_update, scale_update, center_eigen, center_eigen);
        cv::eigen2cv(center_eigen, center);
        // mp_after.push_back(center_eigen);
        cv::Mat center_sim3;
        center.convertTo(center_sim3, CV_32F);
        mps_all[i]->SetWorldPos(center_sim3);
        mps_all[i]->UpdateNormalAndDepth();
    }

    // update keyframe pose use sim3
    for (int i = 0; i < vpKFs.size(); i++) {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
        while (pKF->isBad()) {
            continue;
        }
        Eigen::Matrix4d keyframe_pose = Eigen::Matrix4d::Identity();
        cv::Mat Trw = pKF->GetPose();
        cv::Mat Rwc = Trw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Trw.rowRange(0, 3).col(3);
        Eigen::Matrix3d rot;
        Eigen::Vector3d trans;
        cv::cv2eigen(Rwc, rot);
        cv::cv2eigen(twc, trans);
        keyframe_pose.block(0, 0, 3, 3) = rot;
        keyframe_pose.block(0, 3, 3, 1) = trans;
        TransformPoseUseSim3(T12_update, scale_update, keyframe_pose, keyframe_pose);
        // local_poses[count++] = keyframe_pose;
        cv::Mat Twc;
        cv::eigen2cv(keyframe_pose, Twc);
        cv::Mat Rcw = Twc.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat tcw = -Rcw * Twc.rowRange(0, 3).col(3);
        cv::Mat Tcw_sim3 = cv::Mat::eye(4, 4, CV_32F);
        for (int l = 0; l < 3; l++) {
            for (int m = 0; m < 4; m++) {
                if (m < 3) {
                    Tcw_sim3.at<float>(l, m) = Rcw.at<double>(l, m);
                } else {
                    Tcw_sim3.at<float>(l, m) = tcw.at<double>(l);
                }
            }
        }
        pKF->SetPose(Tcw_sim3);
        pKF->UpdateConnections();
    }

    // update current frame
    TransformPoseUseSim3(T12_update, scale_update, frame_pose, frame_pose);

    align_init = true;
}

bool VisualLocalization::CheckAlign(std::vector<Eigen::Vector3d>& global,
                                    std::vector<Eigen::Vector3d>& local,
                                    const Eigen::Matrix4d& T12,
                                    const double& scale,
                                    const double& threshold,
                                    bool need_consistency)
{
    double residual = 0.0;
    int size_position = local.size();
    bool b_check = false;
    if (size_position <= 0) {
        UpdateAlignHistory(b_check);
        return false;
    }
    if (isinf(scale)) {
        UpdateAlignHistory(b_check);
        return false;
    }
    // LOG(INFO)<<"T12: "<<T12<<"  scale: "<<scale;
    for (int i = 0; i < size_position; i++) {
        Eigen::Vector3d local_to_global;
        TransformPositionUseSim3(T12, scale, local[i], local_to_global);
        double tmp = sqrt(pow(global[i][0] - local_to_global[0], 2) +
                          pow(global[i][1] - local_to_global[1], 2) +
                          pow(global[i][2] - local_to_global[2], 2));
        residual += tmp;
    }
    residual /= size_position;
    LOG(INFO) << "residual: " << residual << " CheckAlign: "
              << (residual < threshold || (!align_init && residual < threshold * 1.5));
    if (residual < threshold || (!align_init && residual < threshold * 1.5)) {
        b_check = true;
    }

    if (!need_consistency) {
        return b_check;
    }

    bool check_align = UpdateAlignHistory(b_check);

    return check_align;
}

bool VisualLocalization::UpdateAlignHistory(bool b_check)
{
    check_align_history[0] = check_align_history[1];
    check_align_history[1] = check_align_history[2];
    check_align_history[2] = b_check;

    for (int i = 0; i < 3; i++) {
        if (!check_align_history[i]) {
            return false;
        }
    }

    return true;
}

void VisualLocalization::TransformPoseUseSim3(const Eigen::Matrix4d& sim3,
                                              const double scale,
                                              Eigen::Matrix4d& in_pose,
                                              Eigen::Matrix4d& out_pose)
{
    Eigen::Matrix3d R_tran = sim3.block(0, 0, 3, 3) / scale;
    Eigen::Matrix3d R_in = in_pose.block(0, 0, 3, 3);
    Eigen::Matrix3d R_out = R_tran * R_in;
    Eigen::Vector4d t_out = sim3 * in_pose.block(0, 3, 4, 1);
    out_pose = Eigen::Matrix4d::Identity();
    out_pose.block(0, 0, 3, 3) = R_out;
    out_pose.block(0, 3, 4, 1) = t_out;
}

void VisualLocalization::TransformPositionUseSim3(const Eigen::Matrix4d& sim3,
                                                  const double scale,
                                                  Eigen::Vector3d& in_position,
                                                  Eigen::Vector3d& out_position)
{
    Eigen::Vector4d in_tmp(in_position(0), in_position(1), in_position(2), 1);
    Eigen::Vector4d out_tmp = sim3 * in_tmp;
    out_position(0) = out_tmp(0);
    out_position(1) = out_tmp(1);
    out_position(2) = out_tmp(2);
}

void VisualLocalization::GetNNearestKF(int frame_id,
                                       int N,
                                       std::vector<ORB_SLAM2::KeyFrame*>& vpKFs,
                                       std::vector<ORB_SLAM2::KeyFrame*>& covisible_kf)
{
    std::vector<int> kf_number;
    int min_frame_id = frame_id;
    for (int i = 0; i < vpKFs.size(); i++) {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
        if (get_frame_id(pKF->file_name_) < min_frame_id)
            kf_number.push_back(i);
    }
}


bool VisualLocalization::ComputeSim3Ransac(std::vector<Eigen::Vector3d>& P1,
                                           std::vector<Eigen::Vector3d>& P2,
                                           Eigen::Matrix4d& T12,
                                           double& scale_12)
{ 
    std::vector<size_t> all_indices;
    int match_size = P1.size();
    all_indices.reserve(match_size);
    for (int i = 0; i < match_size; i++) {
        all_indices.push_back(i);
    }

    float ransac_prob = 0.99;
    int ransac_min_inliers = max(30,int(match_size * 0.3));
    int ransac_max_iter = 300;

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)ransac_min_inliers / match_size;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int n_iterations;

    if (ransac_min_inliers == match_size)
        n_iterations = 1;
    else
        n_iterations = ceil(log(1 - ransac_prob) / log(1 - pow(epsilon, 3)));

    ransac_max_iter = max(1, min(n_iterations, ransac_max_iter));

    int count_iter = 0;

    if (match_size < ransac_min_inliers) {
        return false;
    }

    while (count_iter < ransac_max_iter) {
        count_iter++;

        std::vector<size_t> available_indices;
        available_indices = all_indices;

        std::vector<Eigen::Vector3d> P3D1, P3D2;
        P3D1.reserve(3);
        P3D2.reserve(3);
        // Get min set of points
        for (short i = 0; i < 3; ++i) {
            int randi = DUtils::Random::RandomInt(0, available_indices.size() - 1);

            int idx = available_indices[randi];
            P3D1.push_back(P1[idx]);
            P3D2.push_back(P2[idx]);

            available_indices[randi] = available_indices.back();
            available_indices.pop_back();
        }

        orb_slam::ComputeSim3(P3D1, P3D2, T12, scale_12);
        //LOG(INFO)<< "scale_12:"<<scale_12;  

        std::vector<bool> b_inliners = std::vector<bool>(match_size,false);
        int ninliers = CheckInliers(P1, P2, T12, scale_12, b_inliners);

        //LOG(INFO)<< ransac_min_inliers<<"  "<<ninliers;  
        if (ninliers > ransac_min_inliers) {
            std::vector<Eigen::Vector3d> P3D1_inlier, P3D2_inlier;
            P3D1_inlier.reserve(match_size);
            P3D2_inlier.reserve(match_size);
            for (int i = 0; i < match_size; i++){
                if (b_inliners[i]){
                    P3D1_inlier.push_back(P1[i]);
                    P3D2_inlier.push_back(P2[i]);
                }
            }
            orb_slam::ComputeSim3(P3D1_inlier, P3D2_inlier, T12, scale_12);
            int ninliers_final = CheckInliers(P3D1_inlier, P3D2_inlier, T12, scale_12, b_inliners);
            if(ninliers_final > 0.9 * ninliers){
                return true;
            }
        }
    }

    return false;
}

int VisualLocalization::CheckInliers(std::vector<Eigen::Vector3d>& P1,
                                     std::vector<Eigen::Vector3d>& P2,
                                     const Eigen::Matrix4d& T12,
                                     const double& scale,
                                     std::vector<bool> &b_inliners)
{
    int size_position = P1.size();
    int inlier = 0;
    if (isinf(scale)) {
        return inlier;
    }
    for (int i = 0; i < size_position; i++) {
        Eigen::Vector3d local_to_global;
        TransformPositionUseSim3(T12, scale, P2[i], local_to_global);
        double tmp = sqrt(pow(P1[i][0] - local_to_global[0], 2) +
                          pow(P1[i][1] - local_to_global[1], 2) +
                          pow(P1[i][2] - local_to_global[2], 2));
        if(tmp < 2.0){
            b_inliners[i] = true;
            inlier++;
        }
    }

    return inlier;
}
}  // namespace visual_loc
