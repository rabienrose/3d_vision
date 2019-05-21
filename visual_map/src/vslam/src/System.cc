#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"


namespace ORB_SLAM2
{
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
    
    System::System(const string &strVocFile, const string &strSettingsFile)
    {
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad= mpVocabulary->loadFromBinaryFile(strVocFile);
        if(bVocLoad==false){
            std::cout<<"try binary voc failed, use txt format to load."<<std::endl;
            mpVocabulary->load(strVocFile);
        }
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        mpMap = new Map();
        mpTracker = new Tracking(mpVocabulary, mpMap, mpKeyFrameDatabase, strSettingsFile,0 ,false);
        mpLocalMapper = new LocalMapping(mpMap, true);
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, false);
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
        last_kfcount=0;
    }
    
    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp, std::string file_name)
    {
        cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp, file_name);
        return Tcw;
    }
    
    void System::getTraj(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& quas){
        std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
            while(pKF->isBad())
            {
                continue;
            }
            cv::Mat Two = vpKFs[0]->GetPoseInverse();
            cv::Mat Trw = pKF->GetPose()*Two;
            cv::Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Trw.rowRange(0,3).col(3);
            //Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(2),-twc.at<float>(1));
            Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2));
            posis.push_back(posi);
            Eigen::Quaterniond rot(ORB_SLAM2::Converter::toMatrix3d(Rwc));
            quas.push_back(rot);
        }
    }
    
    void System::getPC(std::vector<Eigen::Vector3d>& pcs){
        std::vector<MapPoint*> vpMPs= mpMap->GetAllMapPoints();
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* pMP=vpMPs[i];
            if(pMP->isBad()){
                continue;
            }
            Eigen::Vector3d posi = Converter::toVector3d(pMP->GetWorldPos());
            pcs.push_back(posi);
        }
    }
    
    void System::getDebugImg(cv::Mat& img, float& err, int& count, int & mp_count_, int& kf_count_){
        err=0;
        count=0;
        img=cv::Mat();
        std::vector<MapPoint*> vpMPs= mpMap->GetAllMapPoints();
        std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        mp_count_=vpMPs.size();
        kf_count_=vpKFs.size();
        if(mpTracker->created_new_kf){
            KeyFrame* lastKF = mpTracker->last_kf;
            img=mpTracker->mKfImage;
            cv::Mat img_rgb;
            cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2BGRA);
            std::vector<MapPoint*> mps= lastKF->GetMapPointMatches();
            double total_error=0;
            for(int i=0; i<mps.size(); i++){
                if(mps[i]!=NULL){
                    if(!mps[i]->isBad()){
                        cv::Mat pose = lastKF->GetPose();
                        Eigen::Matrix4d pose_eig= Converter::toMatrix4d(pose);
                        Eigen::Vector4d posi_mp;
                        cv::Mat posi_mp_cv = mps[i]->GetWorldPos();
                        posi_mp[0]=posi_mp_cv.at<float>(0);
                        posi_mp[1]=posi_mp_cv.at<float>(1);
                        posi_mp[2]=posi_mp_cv.at<float>(2);
                        posi_mp[3]=1;
                        Eigen::Matrix<double, 3, 4> K_eig=Eigen::Matrix<double, 3, 4>::Zero();
                        K_eig(0,0)=lastKF->fx;
                        K_eig(1,1)=lastKF->fy;
                        K_eig(0,2)=lastKF->cx;
                        K_eig(1,2)=lastKF->cy;
                        K_eig(2,2)=1;
                        Eigen::Vector3d projected_pt = K_eig*pose_eig*posi_mp;
                        cv::Point2f obs_pt(projected_pt(0)/projected_pt(2), projected_pt(1)/projected_pt(2));
                        int kp_index = mps[i]->GetIndexInKeyFrame(lastKF);
                        cv::Point2f pt= lastKF->mvKeysUn[kp_index].pt;
                        cv::Point2f d_pt=obs_pt-pt;
                        //std::cout<<pt.x<<","<<pt.y<<" "<<obs_pt.x<<","<<obs_pt.y<<std::endl;
                        total_error=total_error+sqrt(d_pt.x*d_pt.x + d_pt.y*d_pt.y);
                        cv::circle(img_rgb, obs_pt ,3 ,cv::Scalar(0, 255, 255), 1, 1 ,0);
                        count++;
                    }
                }
            }
            err=total_error/count;
            img=img_rgb;
        }
    }
    
    size_t findDesc(std::vector<std::pair<KeyFrame*, size_t>>& target_list, std::pair<KeyFrame*, size_t> query){
        size_t re=-1;
        for(int i=0; i<target_list.size(); i++){
            if(query.first==target_list[i].first && query.second==target_list[i].second){
                if(!target_list[i].first->isBad()){
                    re=i;
                }else{
                    re=0;
                }
                //std::cout<<(int)re<<std::endl;
                break;
            }
        }
        return re;
    }
    
    void System::saveResult(string map_filename){
        string track_file=map_filename+"/track.txt";
        string desc_file=map_filename+"/desc.txt";
        string kp_file=map_filename+"/kps.txt";
        string posi_file=map_filename+"/posi.txt";
        string traj_file=map_filename+"/traj.txt";
        
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(traj_file.c_str());
        
        int count=0;

        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

            while(pKF->isBad())
            {
                continue;
            }

            cv::Mat Trw = pKF->GetPose()*Two;

            cv::Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Trw.rowRange(0,3).col(3);
            
            std::vector<std::string> splited = split(pKF->file_name_, "/");
            std::string filename= splited.back();
            std::stringstream ss;
            f<<filename<<",";

            f << pKF->mnId<<","<< Rwc.at<float>(0,0) << "," << Rwc.at<float>(0,1)  << "," << Rwc.at<float>(0,2) << ","  << twc.at<float>(0) << "," <<
                Rwc.at<float>(1,0) << "," << Rwc.at<float>(1,1)  << "," << Rwc.at<float>(1,2) << ","  << twc.at<float>(1) << "," <<
                Rwc.at<float>(2,0) << "," << Rwc.at<float>(2,1)  << "," << Rwc.at<float>(2,2) << ","  << twc.at<float>(2) << endl;
            
            count++;
        }
        f.close();

        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        std::vector<cv::Mat> posis;
        
        int desc_count=0;
        int track_count=0;
        std::vector<std::pair<KeyFrame*, size_t>> desc_list;
        std::vector<std::vector<size_t>> track_list;
        std::vector<cv::Mat> posi_list;
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* mp = vpMPs[i];
            if (mp->isBad()){
                continue;
            }
            map<KeyFrame*, size_t> tracks= mp->GetObservations();
            std::vector<size_t> track_out;
            for(auto item: tracks){
                size_t desc_id = findDesc(desc_list, item);
                if(desc_id!=0){
                    if(desc_id==-1){
                        desc_list.push_back(item);
                        track_out.push_back(desc_list.size()-1);
                    }else{
                        track_out.push_back(desc_id);
                    }
                }
                
            }
            if(track_out.size()>=3){
                track_list.push_back(track_out);
                posi_list.push_back(vpMPs[i]->GetWorldPos());
            }
        }
        std::cout<<"desc_list: "<<desc_list.size()<<std::endl;
        std::cout<<"track_list: "<<track_list.size()<<std::endl;
        f.open(track_file.c_str());
        //f << fixed;
        for(int i=0; i<track_list.size(); i++){
            for (auto track: track_list[i]){
                f<<track<<",";
            }
            f<<std::endl;
        }
        f.close();
        f.open(posi_file.c_str());
        for(int i=0; i<posi_list.size(); i++){
            for(int j=0; j<3; j++){
                f<<posi_list[i].at<float>(j)<<",";
            }
            f<<std::endl;
        }
        f.close();
        f.open(kp_file.c_str());
        for(auto desc: desc_list){
            
            cv::KeyPoint kp = desc.first->mvKeysUn[desc.second];
            f<<kp.pt.x<<","<<kp.pt.y<<","<<kp.octave<<","<<desc.first->file_name_;
            f<<std::endl;
        }
        f.close();
        f.open(desc_file.c_str());
        for(auto desc: desc_list){
            
            cv::Mat desc_mat = desc.first->mDescriptors.row(desc.second);
            //std::cout<<desc_mat<<std::endl;
            for(int i=0; i<desc_mat.cols; i++){
                f<<(int)desc_mat.at<unsigned char>(0,i)<<",";
            }
            f<<std::endl;
        }
        f.close();
    }
} //namespace ORB_SLAM
