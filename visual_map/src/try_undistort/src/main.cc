#include <iostream>
#include <dirent.h>
#include "opencv2/opencv.hpp"

int main(int argc, char** argv){
    
//       distortion_coeffs: [-0.09825993,  0.05008594, -0.00012027, -0.00029007]
//   distortion_model: radtan
//   intrinsics: [541.39699791,  540.86222539,  474.6630494,   306.8632145]
    
    cv::Mat cam_m(3, 3, CV_32FC1);
    cam_m.at<float>(0,0)=541.39699791;
    cam_m.at<float>(1,1)=540.86222539;
    cam_m.at<float>(0,2)=474.6630494;
    cam_m.at<float>(1,2)=306.8632145;
    cam_m.at<float>(2,2)=1;
    cv::Mat cam_dis(4, 1, CV_32FC1);
    cam_dis.at<float>(0,0)=-0.09825993;
    cam_dis.at<float>(1,0)=0.05008594;
    cam_dis.at<float>(2,0)=-0.00012027;
    cam_dis.at<float>(3,0)=-0.00029007;
    std::cout<<cam_m<<std::endl;
    std::cout<<cam_dis<<std::endl;
    std::string img_addr="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/camera_1_img/img_279.jpg";
    cv::Mat img = cv::imread(img_addr);
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    cv::Mat img_undistort;
    cv::undistort(img, img_undistort, cam_m, cam_dis);
    cv::imwrite("./try_undistort.jpg", img_undistort);
    return 0;
}
