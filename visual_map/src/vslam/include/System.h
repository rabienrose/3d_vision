/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H
#include<opencv2/core/core.hpp>
#include "Tracking.h"
#include<string>
#include<thread>



#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Eigen/Core"
namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{


public:
    System(const string &strVocFile, const string &strSettingsFile);
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp, std::string file_name="");
    void getPC(std::vector<Eigen::Vector3d>& pcs);
    void getTraj(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& quas);
    void getDebugImg(cv::Mat& img, float& err, int& count, int & mp_count_, int& kf_count_);
private:
    ORBVocabulary* mpVocabulary;
    KeyFrameDatabase* mpKeyFrameDatabase;
    Map* mpMap;
    Tracking* mpTracker;
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopCloser;
    int last_kfcount;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
