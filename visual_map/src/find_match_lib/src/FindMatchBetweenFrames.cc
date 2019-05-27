#include "FindMatchBetweenFrames.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <glog/logging.h>
#include <sys/stat.h>
#include <sys/types.h> 


void drawKeypointNumber(cv::Mat& image,const cv::KeyPoint& p,const int number)
{
    cv::putText(image,std::to_string(number),cvPoint(p.pt.x,p.pt.y),
                CV_FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0),1,4);    
}
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
namespace find_match{
 
    FindMatchBetweenFrames::FindMatchBetweenFrames()
    {

    }
    FindMatchBetweenFrames::~FindMatchBetweenFrames()
    {}

    void FindMatchBetweenFrames::get_matches_from_bag(std::string& bag_dir, 
                                                      std::string& bag_name,
                                                      std::string& img_topic, 
                                                      cv::Mat cam_matrix, cv::Mat cam_distort)
    {
        get_pose(bag_dir);
        rosbag::Bag bag;
        std::string bag_file = bag_dir + "/" + bag_name;
        bag.open(bag_file,rosbag::bagmode::Read);  
        rosbag::View view(bag, rosbag::TopicQuery(img_topic));
        cv::Mat img_undistort;
        
        cam_mat = cam_matrix;
        rosbag::View::iterator it= view.begin();
        int frame_count = view.size();
        LOG(INFO) << frame_count << " frames to be processed";
        kps_list_in_all_frames.resize(frame_count);
        desc_list_in_all_frames.resize(frame_count);
        mgrid_in_all_frames.resize(frame_count);
        kps_info_list_in_all_frames.reserve(frame_count);
        int frame_id = 0;
        dir_name = bag_dir+"/img/";
        mkdir(dir_name.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        int  key_frame_id = 0;
        for(const auto& m : view)
        {
            sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
            if(!simg.get()) 
            {
                continue;
            }
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat desc_list;
            cv::Mat mask;
            // std::vector<cv::KeyPoint> kps_list;

            // mgrid_info mgrid;
            try
            {
                std::vector<std::shared_ptr<KpInfo>> kp_info_list;
                kp_info_list.reserve(2000);
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                cv::Mat img_distort = cv_ptr->image;
                cv::cvtColor(img_distort, img_distort, cv::COLOR_BGR2GRAY);
                cv::undistort(img_distort, img_undistort, cam_matrix, cam_distort);
                std::stringstream img_name;
                img_name<<dir_name<<"img_"<<frame_id<<".jpg";
                cv::imwrite(img_name.str(), img_undistort);
                extract_orb(img_undistort, mask, desc_list_in_all_frames[frame_id],
                            kps_list_in_all_frames[frame_id], mgrid_in_all_frames[frame_id]);
                bool is_key_frame = find_nearest_key_frame(frame_id,key_frame_id);
                if(key_frame_id < 0) 
                    continue;
                // std::cout<<"frame_id: "<<frame_id<<" is_key_frame: "<<is_key_frame<<std::endl;
                for(size_t i = 0; i < kps_list_in_all_frames[frame_id].size(); i++)
                {
                    // std::shared_ptr<KpInfo> kp ;
                    // kp.kp = &kps_list_in_all_frames[frame_id][i];
                    // kp.desc = desc_list_in_all_frames[frame_id].row(i);
                    // kp.track_id = 0;
                    // kp.frame_id = frame_id;
                    // kp.is_key_frame = is_key_frame;

                    kp_info_list.emplace_back(std::make_shared<KpInfo>(
                        &kps_list_in_all_frames[frame_id][i],
                        desc_list_in_all_frames[frame_id].row(i), frame_id, 0,
                        is_key_frame));
                }
                kps_info_list_in_all_frames.emplace_back(key_frame_id, std::move(kp_info_list));
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            frame_id++;
        }
        width  = img_undistort.cols;
        height = img_undistort.rows*0.7;
        LOG(INFO) << "ExtractOrb Done";

        find_tracks();
        LOG(INFO) << "Find Tracks Done";

        write_tracks(bag_dir);
    }
    
    void FindMatchBetweenFrames::get_matches_from_images(std::string& image_1, 
                                                      std::string& image_2,
                                                      cv::Mat cam_matrix, cv::Mat cam_distort)
    {

        cv::Mat img_undistort;
        cv::Mat img_1,img_2;
        cam_mat = cam_matrix;
        
        img_undistort = cv::imread(image_1,-1);
        cv::Mat mask = cv::Mat::zeros(img_undistort.size(), CV_8UC1);
        cv::Mat dst;
        std::vector<std::vector<cv::Point2i>> contours;
        std::vector<cv::Point2i> points;
        points.push_back(cv::Point2i(230, 100));
        points.push_back(cv::Point2i(800, 100));
        points.push_back(cv::Point2i(600, 350));
        points.push_back(cv::Point2i(400, 350));

        contours.push_back(points);
        cv::drawContours(mask, contours, 0, cv::Scalar::all(255), -1);

        // img_undistort.copyTo(dst, mask);
        img_undistort.copyTo(dst);
        // cv::imshow("dst", dst);
        // cv::waitKey(0);
        cv::undistort(dst, img_1, cam_matrix, cam_distort);
        img_undistort = cv::imread(image_2,-1);
        // img_undistort.copyTo(dst, mask);
        img_undistort.copyTo(dst);
        cv::undistort(dst, img_2, cam_matrix, cam_distort);

        std::vector<cv::KeyPoint> kps_list_1, kps_list_2;
        cv::Mat desc_1, desc_2;
        mgrid_info mgrid_1,mgrid_2;
        std::vector<std::pair<int, int>> match12;
        orb_slam::ExtractOrb(img_1, desc_1, kps_list_1, mgrid_1,cam_matrix,cam_distort);
        orb_slam::ExtractOrb(img_2, desc_2, kps_list_2, mgrid_2,cam_matrix,cam_distort);
        width  = img_1.cols;
        height = img_1.rows;
        orb_slam::FindMatchInTwoFrame(kps_list_1, kps_list_2, desc_1, desc_2, width, height,
                mgrid_2,match12,cam_matrix);
        std::vector<cv::DMatch> good_matches;
        good_matches.reserve(match12.size());
        std::transform(
                match12.cbegin(), match12.cend(),std::back_inserter(good_matches),
                [](const std::pair<int, int>& match) {
                return cv::DMatch(match.first, match.second, FLT_MAX);
                });
        cv::Mat show_match;
        for(int i = 0; i < 100; i++)
        {
            drawKeypointNumber(img_1,kps_list_1[i],i);                
        }
        // cv::imshow("img1",img_1);
        // cv::waitKey(0);
        // for(int i = 650; i < 690; i++)
        // {
            // drawKeypointNumber(img_2,kps_list_2[i],i);                
        // }
        // cv::imshow("img2",img_2);
        // cv::waitKey(0);
        cv::drawMatches(img_1,kps_list_1,img_2,kps_list_2,good_matches,show_match);
        cv::resize(show_match, show_match, cv::Size(1920, 1080), (0, 0), (0, 0), cv::INTER_LINEAR);
        cv::imshow("Matches",show_match);
        cv::waitKey(0);

    }
    
    void FindMatchBetweenFrames::find_tracks()
    {
        int nframe = desc_list_in_all_frames.size();
        int track_count = 0;
        for(int frame_id_1 = 0; frame_id_1 < nframe; frame_id_1++)
        {
            LOG_EVERY_N(INFO,5) << "Processing "<< frame_id_1 <<" frame";
            std::vector<cv::KeyPoint> kps_list_1, kps_list_2;
            cv::Mat desc_1, desc_2;
            mgrid_info mgrid_2;
            kps_list_1 = kps_list_in_all_frames[frame_id_1];
            desc_1 = desc_list_in_all_frames[frame_id_1];

            std::vector<std::shared_ptr<KpInfo>> &kp_info_list_1 =
                kps_info_list_in_all_frames[frame_id_1].second;
            int key_frame_id_1 =  kps_info_list_in_all_frames[frame_id_1].first;
            Eigen::Quaterniond ori_1 = key_frame_poses[key_frame_id_1].ori;
            Eigen::Vector3d  pos_1 = key_frame_poses[key_frame_id_1].pos;
            std::stringstream img_name_1;
            img_name_1<<dir_name<<"img_"<<frame_id_1<<".jpg";
            cv::Mat img_1 = cv::imread(img_name_1.str(),-1);

            // for(int i = 0; i < 100; i++)
            // {
                // drawKeypointNumber(img_1,kps_list_1[i],i);                
            // }
            // cv::imshow("img1",img_1);
            // cv::waitKey(0);
            for (int frame_id_2 = frame_id_1 + 1;
                 frame_id_2 <= frame_id_1 + 1 && frame_id_2 < nframe;
                 frame_id_2++) {
              int key_frame_id_2 =
                  kps_info_list_in_all_frames[frame_id_2].first;
              Eigen::Quaterniond ori_2 = key_frame_poses[key_frame_id_2].ori;
              Eigen::Vector3d pos_2 = key_frame_poses[key_frame_id_2].pos;
              float dis = 0.0;
              for (int i = 0; i < 3; i++) {
                dis += pow(pos_1[i] - pos_2[i], 2);
              }
              // std::cout<<"dis: "<<dis<<std::endl;
              // if (sqrt(dis) > 1)
                // continue;

              kps_list_2 = kps_list_in_all_frames[frame_id_2];
              desc_2 = desc_list_in_all_frames[frame_id_2];
              mgrid_2 = mgrid_in_all_frames[frame_id_2];
              std::vector<std::shared_ptr<KpInfo>> &kp_info_list_2 =
                  kps_info_list_in_all_frames[frame_id_2].second;
              std::vector<std::pair<int, int>> match12;
              std::stringstream img_name_2;
              img_name_2 << dir_name << "img_" << frame_id_2 << ".jpg";
              cv::Mat img_2 = cv::imread(img_name_2.str(), -1);
              std::cout << "frame_id: " << frame_id_1 << "  " << frame_id_2 << std::endl;
              orb_slam::FindMatchInTwoFrame(kps_list_1, kps_list_2, desc_1,
                                            desc_2, width, height, mgrid_2,
                                            match12, cam_mat);
              std::vector<cv::DMatch> good_matches;
              good_matches.resize(match12.size());
              // for(const auto& match : match12)
              // {
              // good_matches.emplace_back(match.first,match.second,FLT_MAX);
              // }
              std::transform(
                  match12.cbegin(), match12.cend(), good_matches.begin(),
                  [](const std::pair<int, int> &match) {
                    return cv::DMatch(match.first, match.second, FLT_MAX);
                  });
              // cv::Mat show_match;
              // cv::drawMatches(img_1,kps_list_1,img_2,kps_list_2,good_matches,show_match);
              // cv::resize(show_match, show_match, cv::Size(1920, 1080), (0,
              // 0), (0, 0), cv::INTER_LINEAR);
              // cv::imshow("Matches",show_match);
              // cv::waitKey(0);
              if (match12.size() < 10)
                continue;
              for (size_t i = 0; i < match12.size(); i++) {
                int kp1 = match12[i].first;
                int kp2 = match12[i].second;
                auto &track_id_1 = kp_info_list_1[kp1]->track_id;
                auto &track_id_2 = kp_info_list_2[kp2]->track_id;
                // if(track_id_1 == 0 && track_id_2 != 0)
                    LOG(ERROR) << "track_id " << track_id_1 << "  " << track_id_2;

                if (track_id_1 == 0 && track_id_2 == 0) {
                  track_count++;  
                  track_id_1 = track_count;
                  track_id_2 = track_count;
                  kp_track_pairs[track_count].push_back(kp_info_list_1[kp1]);
                  kp_track_pairs[track_count].push_back(kp_info_list_2[kp2]);
                } else if (track_id_1 != 0 && track_id_2 == 0) {
                  track_id_2 = track_id_1;
                  kp_track_pairs[track_id_2].push_back(kp_info_list_2[kp2]);

                } else if (track_id_1 == 0 && track_id_2 != 0) {
                  track_id_1 = track_id_2;
                  kp_track_pairs[track_id_1].push_back(kp_info_list_1[kp1]);
                } else {
                  auto kps = kp_track_pairs[track_id_2];
                  for (auto &kp : kps) {
                    kp->track_id = track_id_1;
                    kp_track_pairs[track_id_1].push_back(kp);
                  }
                  auto iter = kp_track_pairs.find(track_id_2);
                  kp_track_pairs.erase(iter);
                  // continue;
                }
              }
            }
        }
    }

    void FindMatchBetweenFrames::write_tracks(std::string& output_dir)
    {
        std::ofstream track_file,kps_file,desc_file;
        track_file.open(output_dir + "/track.txt");
        kps_file.open(output_dir + "/kps.txt");
        desc_file.open(output_dir + "/desc.txt");

        if(!track_file.is_open() || !kps_file.is_open() || !desc_file.is_open())
        {
            LOG(ERROR) << "track kps or desc_file file is not open!";
        }
        int kps_count = 0;
        for(const auto& it : kp_track_pairs)
        {
            std::stringstream track_string;
            if(it.second.size() < 3) continue;
            for(const auto& kps : it.second)
            {
                if(!kps->is_key_frame)
                    continue;
                std::stringstream kps_string;
                std::stringstream desc_string;
                kps_string << kps->kp->pt.x << "," << kps->kp->pt.y << ",";
                kps_string << kps->kp->octave << ",img_" << kps->frame_id <<".jpg"<< std::endl;
                for(int i = 0; i < kps->desc.cols; i++)
                 {
                     desc_string<<(int)kps->desc.at<unsigned char>(0,i)<<",";
                 }
                desc_string<< std::endl;
                kps_file << kps_string.str();
                desc_file << desc_string.str();
                track_string << kps_count++ << ",";
            }
            track_string << std::endl;
            track_file << track_string.str();
        }

        track_file.close();
        kps_file.close();
        desc_file.close();
    }
    
    void FindMatchBetweenFrames::extract_orb(cv::Mat img, cv::Mat mask, cv::Mat& desc_list, std::vector<cv::KeyPoint>& kps_list,
                    mgrid_info& mGrid){
        
        int features_count=2000;
        float features_scale_rate=1.2;
        int features_level=8;
        int ini_cell_fast=20;
        int min_cell_fast=7;
        ORB_SLAM2::ORBextractor extractor(features_count,features_scale_rate,features_level,
                                            ini_cell_fast,min_cell_fast);
        
        extractor.ExtractDesc(img, mask, kps_list, desc_list, true);
        int width = img.cols;
        int height = img.rows;
        orb_slam::CalGrid(kps_list,width,height,mGrid);
    }

    void FindMatchBetweenFrames::get_pose(std::string& posi_dir)
    {
        std::string line;
        std::ifstream infile_lidar(posi_dir+"/traj_alin.txt");
        while (true)
        {
            std::getline(infile_lidar, line);
            if (line==""){
                break;
            }
            KFInfo frame;
            std::vector<std::string> splited = split(line, ",");
            Eigen::Vector3d posi;
            posi.x()=atof(splited[5].c_str());
            posi.y()=atof(splited[9].c_str());
            posi.z()=atof(splited[13].c_str());
            
            Eigen::Matrix3d rot;
            rot << atof(splited[2].c_str()), atof(splited[3].c_str()), atof(splited[4].c_str()),
                   atof(splited[6].c_str()), atof(splited[7].c_str()), atof(splited[8].c_str()),
                   atof(splited[10].c_str()),atof(splited[11].c_str()),atof(splited[12].c_str());
            Eigen::Quaterniond qua(rot);
            frame.pos = posi;
            frame.ori  = rot;

            std::vector<std::string> splited1 = split(splited[0].c_str(), "_");
            std::vector<std::string> splited2 = split(splited1[1].c_str(), ".");
            frame.frame_id = atoi(splited2[0].c_str());
            key_frame_poses.push_back(frame);
        }
    }

    bool FindMatchBetweenFrames::find_nearest_key_frame(int frame_id,int& key_frame_id)
    {
        bool is_key_frame = false;
        int nkey_frame = key_frame_poses.size(); 
        int i;
        for(i = key_frame_id; i < nkey_frame; i++)
        {
            if(frame_id >= key_frame_poses[i].frame_id && frame_id < key_frame_poses[i+1].frame_id)
                break;
        }
        if(i == nkey_frame)
        {
            key_frame_id = -1;
            return false;
        }
        key_frame_id = i;
        if(frame_id == key_frame_poses[key_frame_id].frame_id)
        {
            is_key_frame = true;
        }

        return is_key_frame;
    }
}
