#ifndef LOCALIZATION_API_H_
#define LOCALIZATION_API_H_

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace wayz {

class LocalizationAPI {
public:

    virtual ~LocalizationAPI() {};
    
    virtual void StartLocalization(const std::string& filename) = 0;
    
    virtual void AddGPS(const double timestamp, const Eigen::Vector3d& LatLonHei) = 0;//LatLonHei means Lattitude Longitude and Height
  
    virtual void AddImage(const double timestamp,const int camera_id, const cv::Mat& Img)  = 0;
    
    virtual void AddIMU(const double timestamp, const Eigen::Vector3d& Accl, const Eigen::Vector3d& Gyro)  = 0;

    virtual void AddMap(std::stringstream& pointer_address) = 0;

    virtual void AddMap(const std::string& folder_path) = 0;
    
    virtual void Shutdown() = 0;

    virtual bool QueryPose(const double timestamp, Eigen::Vector3d& Pos, Eigen::Vector3d& Vel, Eigen::Quaterniond& Ori) const = 0;

};

}  // namespace rovioli

#endif  // LOCALIZATION_API_H_
