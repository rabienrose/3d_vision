#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

int main(int argc, char **argv) {
    std::string res_root=argv[1];
    std::string pcd_name=argv[2];
    float rate=atof(argv[3]);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud_g(new pcl::PointCloud<pcl::PointXYZ>);
    
    if ( -1 == pcl::io::loadPCDFile<pcl::PointXYZ>(res_root+pcd_name, *inputCloud)){
        PCL_ERROR("Couldn't read this file!");
        return -1;
    }
    
//     pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
//     voxel_filter.setLeafSize(rate,rate,rate);
//     voxel_filter.setInputCloud(inputCloud);
//     voxel_filter.filter(*outputCloud);
    
    pcl::RandomSample<pcl::PointXYZ> downSizeFilter(true);
    downSizeFilter.setInputCloud (inputCloud);
    downSizeFilter.setSample (inputCloud->size()*rate);
    downSizeFilter.filter(*outputCloud);
    
//     for(int i=0; i<outputCloud->points.size(); i++){
//         if(outputCloud->points[i].z>0){
//             outputCloud_g->points.push_back(outputCloud->points[i]);
//         }
//     }
    
    std::string semi_pcd=res_root+"/ds_pc.pcd";
    pcl::io::savePCDFileBinary(semi_pcd, *outputCloud);
    return 0;
}

