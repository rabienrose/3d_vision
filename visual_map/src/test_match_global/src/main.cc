#include <string>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "visual_map/visual_map.h"
#include "gtest/gtest.h"
#include <math.h>
#include "visual_map/visual_map_seri.h"
#include <nabo/nabo.h>
#include <opencv2/core/eigen.hpp>

typedef Nabo::NearestNeighbourSearch<float> NNSearch;

TEST(MatchGlobalTest, Chamo) { 
    std::string save_addr;
    std::string bag_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/wayz_2018_11_26_localization.bag";
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    
    std::shared_ptr<NNSearch> index;
    std::cout<<bag_addr<<std::endl;
    
    std::vector<std::string> topics;
    topics.push_back("camera/right/image_raw");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    int img_count=100000;

    int max_frame_count=10;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> db_desc;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> query_desc;
    
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
        if(simg!=NULL){
            img_count++;
            if(img_count>100000+max_frame_count){
                break;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                std::shared_ptr<vm::Frame> frame_p;
                frame_p.reset(new vm::Frame);
                double timestamp=simg->header.stamp.toSec(); 
                frame_p->time_stamp=timestamp;
                cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
                std::vector<cv::KeyPoint> keypoints1;
                cv::Mat descriptors1;
                detector->detectAndCompute(cv_ptr->image, cv::noArray(), keypoints1, descriptors1);
                std::cout<<"desc width: "<<descriptors1.rows<<std::endl;
                Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> desc_eigen;
                cv2eigen(descriptors1.t(), desc_eigen);
                query_desc=desc_eigen;
                int old_width=db_desc.cols();
                db_desc.conservativeResize(desc_eigen.rows(), old_width+desc_eigen.cols());
                db_desc.block(0, old_width, db_desc.rows(), desc_eigen.cols())=desc_eigen;
                //descriptors1.cols is the width of descriptors
                //descriptors1.rows is the number of descriptors

            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
    }
    
    index.reset(NNSearch::createKDTreeLinearHeap(db_desc, db_desc.rows(), NNSearch::ALLOW_SELF_MATCH | NNSearch::SORT_RESULTS));
    NNSearch::IndexMatrix indice;
    indice.resize(10,1);
    NNSearch::Matrix dist;
    dist.resize(10,1);
    index->knn(query_desc.col(0), indice, dist, 10);
    std::cout<<query_desc.col(0).transpose()<<std::endl;
    for (int i=0; i<dist.rows(); i++){
        std::cout<<dist(i)<<":"<<indice(i)<<std::endl;
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}