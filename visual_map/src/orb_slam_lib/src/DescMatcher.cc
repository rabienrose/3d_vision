#include "orb_slam_lib/DescMatcher.h"
#include <iostream>
#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<stdint.h>

using namespace std;

namespace orb_slam
{

const int TH_HIGH = 100;
const int TH_LOW = 50;
const int HISTO_LENGTH = 30;



void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;
    int count=a.cols/4;
    for(int i=0; i<count; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel,
    std::vector<cv::KeyPoint> &mvKeysUn, std::vector<std::vector<std::vector<std::size_t>>>& mGrid,
    int mnMinX, int mnMinY, int mnMaxX, int mnMaxY
){
    float mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
    float mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
    vector<size_t> vIndices;
    vIndices.reserve(mvKeysUn.size());

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}
std::vector<bool> GetMatchArea(std::vector<cv::KeyPoint> &mvKeysUn, int mnMinX, int mnMaxX, int mnMinY, int mnMaxY
){
    vector<bool> vIndices;
    vIndices.resize(mvKeysUn.size(),false);

    for(int ix = 0; ix < mvKeysUn.size(); ix++)
    {
        if(mvKeysUn[ix].pt.x > mnMinX && mvKeysUn[ix].pt.x < mnMaxX &&
           mvKeysUn[ix].pt.y > mnMinY && mvKeysUn[ix].pt.y < mnMaxY)
        {
            vIndices[ix] = true;
        }
    }
    return vIndices;
}
int SearchForInitialization(std::vector<cv::KeyPoint> &key1, std::vector<cv::KeyPoint> &key2, 
                            cv::Mat desc1, cv::Mat desc2, float mfNNratio,
                            std::vector<std::vector<std::vector<std::size_t>>>& mGrid,
                            int mnMinX, int mnMinY, int mnMaxX, int mnMaxY, bool mbCheckOrientation,
                            vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(key1.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(key2.size(),INT_MAX);
    vector<int> vnMatches21(key2.size(),-1);
    vector<bool> vIndiceskey1, vIndiceskey2;
    vIndiceskey1 = GetMatchArea(key1,280,700,0,300);
    vIndiceskey2 = GetMatchArea(key2,250,800,0,200);
    int empty_count = 0;
    for(size_t i1=0, iend1=key1.size(); i1<iend1; i1++)
    {
//         if(!vIndiceskey1[i1]) continue;
        cv::KeyPoint kp1 = key1[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1+2,
            key2, mGrid, mnMinX, mnMinY, mnMaxX, mnMaxY);

        if(vIndices2.empty())
        {
          empty_count++;
          continue;
        }

        cv::Mat d1 = desc1.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;
        int bestIdx = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;
//             if(!vIndiceskey2[i2]) continue;

            cv::Mat d2 = desc2.row(i2);

            int dist = DescriptorDistance(d1,d2);
            if(vMatchedDistance[i2]<=dist)
                continue;
            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx=bestIdx2;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
                bestIdx=i2;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<=(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = key1[i1].angle-key2[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }
    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=key2[vnMatches12[i1]].pt;

    std::cout<< "key1: " << key1.size() << " nmatches: " << nmatches<< " empty_count: " <<empty_count<<std::endl;

    return nmatches;
}
int SearchForMultipleMatch(std::vector<cv::KeyPoint> &key1, std::vector<cv::KeyPoint> &key2, 
                            cv::Mat desc1, cv::Mat desc2, float mfNNratio,
                            std::vector<std::vector<std::vector<std::size_t>>>& mGrid,
                            int mnMinX, int mnMinY, int mnMaxX, int mnMaxY, bool mbCheckOrientation,
                            std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = std::vector<int>(key1.size()*3,-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    for(size_t i1=21, iend1=key1.size(); i1<33; i1++)
    {
        cv::KeyPoint kp1 = key1[i1];
        int level1 = kp1.octave;
        if(level1>4)
            continue;

        vector<size_t> vIndices2 = GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1+3,
            key2, mGrid, mnMinX, mnMinY, mnMaxX, mnMaxY);

        if(vIndices2.empty())
          continue;

        cv::Mat d1 = desc1.row(i1);

        int bestDist  = TH_LOW;
        int bestDist2 = TH_LOW;
        int bestDist3 = TH_LOW;
        int bestIdx  = -1;
        int bestIdx2 = -1;
        int bestIdx3 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = desc2.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(dist < bestDist3)
            {
              if(dist < bestDist2)
              {
                if(dist < bestDist)
                {
                  bestDist3=bestDist2;
                  bestDist2=bestDist;
                  bestDist =dist;
                  bestIdx3=bestIdx2;
                  bestIdx2=bestIdx;
                  bestIdx = i2;
                }
                else
                {
                    bestDist3=bestDist2;
                    bestDist2=dist;
                    bestIdx3=bestIdx2;
                    bestIdx2=i2;
                }
                  
              }
              else
              {
                  bestDist3=dist;
                  bestIdx3=i2;
              }
                
            }
        }
//         std::cout<<"bestDist: "<< bestDist<< "  "<<bestDist2<<"  "<<bestDist3<<std::endl;
//         std::cout<<"bestIdx: "<< bestIdx<< "  "<<bestIdx2<<"  "<<bestIdx3<<std::endl;
        vnMatches12[3*i1]   = bestIdx;
        vnMatches12[3*i1+1] = bestIdx2;
        vnMatches12[3*i1+2] = bestIdx3;

    }

    return nmatches;
}
int SearchForMatch(std::vector<cv::KeyPoint> &key1, std::vector<cv::KeyPoint> &key2, 
                            cv::Mat desc1, cv::Mat desc2, float mfNNratio,
                            std::vector<std::vector<std::vector<std::size_t>>>& mGrid,
                            int mnMinX, int mnMinY, int mnMaxX, int mnMaxY, bool mbCheckOrientation,
                            vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12)
{
    vnMatches12 = vector<int>(key1.size(),-1);
    vector<cv::DMatch> matches;
    cv::BFMatcher bfMatcher(cv::NORM_HAMMING);
    bfMatcher.match(desc1, desc2, matches);
    
    double min_dist = 1000, max_dist = 0;
    // 找出所有匹配之间的最大值和最小值
    for (int i = 0; i < desc1.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    // 当描述子之间的匹配大于2倍的最小距离时，即认为该匹配是一个错误的匹配。
    // 但有时描述子之间的最小距离非常小，可以设置一个经验值作为下限
    vector<cv::DMatch> good_matches;
    for (int i = 0; i < desc1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0))
            good_matches.push_back(matches[i]);
    }
    for (int i = 0; i < good_matches.size(); i++)
    {
        vnMatches12[good_matches[i].queryIdx] = good_matches[i].trainIdx;
    }    

//     std::cout<< "key1: " << key1.size() << " nmatches: " << good_matches.size()<<std::endl;
    return good_matches.size();
}
} //namespace ORB_SLAM
