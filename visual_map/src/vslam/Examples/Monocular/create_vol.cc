#include <iostream>
#include <stdio.h>
#include <vector>

#include "TemplatedVocabulary.h"
#include "BowVector.h"
#include "FeatureVector.h"
#include "FORB.h"

#include "ORBmatcher.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

bool debug_mode=false;

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> 
  OrbVocabulary;
  
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}

void loadFeatures( std::vector<std::string>& path_to_images,std::string descriptor, std::vector<std::vector<cv::Mat>> &features) throw (std::exception){
    ORB_SLAM2::ORBextractor mpORBextractor(2000, 1.2, 8, 20, 7);
    for(size_t i = 0; i < path_to_images.size(); ++i)
    {
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> keypoints;
        //std::cout<<"reading image: "<<path_to_images[i]<<std::endl;
        cv::Mat image = cv::imread(path_to_images[i], 0);
        mpORBextractor(image, cv::Mat() ,keypoints, descriptors);
        //cv::Ptr<cv::ORB> orb = cv::ORB::create();
        //orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
        std::vector<cv::Mat> out;
        changeStructure(descriptors, out);
        std::cout<<"img: "<<i<<std::endl;
        features.push_back(out);
    }
}

void testVocCreation(std::vector<std::vector<cv::Mat>> &features)
{
    const int k = 15;
    const int L = 3;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L1_NORM;
    OrbVocabulary voc(k, L, weight, score);
    voc.create(features);
    voc.save("small_voc.yml.gz");
}

std::vector<std::string> get_file_list(std::string img_root){
    std::vector<std::string> file_list_vec;
    for(int i=0; i<4000; i++){
        std::stringstream ss;
        ss << i;
        file_list_vec.push_back(img_root + "/img_"+ss.str() + ".jpg");
    }
    
    return file_list_vec;
}

int main(int argc,char **argv)
{
    std::vector<std::string> file_list_vec = get_file_list(argv[1]);
    try{
        std::vector<std::vector<cv::Mat>> features;
        loadFeatures(file_list_vec, "orb", features);
        testVocCreation(features);
    }catch(std::exception &ex){
        std::cerr<<ex.what()<<std::endl;
    }

    return 0;
}