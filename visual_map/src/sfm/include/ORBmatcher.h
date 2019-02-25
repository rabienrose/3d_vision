#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

namespace orb_slam
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(std::vector<cv::KeyPoint> &key1, std::vector<cv::KeyPoint> &key2, 
                            cv::Mat desc1, cv::Mat desc2, float mfNNratio,
                            std::vector<std::vector<std::vector<std::size_t>>>& mGrid,
                            int mnMinX, int mnMinY, int mnMaxX, int mnMaxY, bool mbCheckOrientation,
                            std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=100);

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
