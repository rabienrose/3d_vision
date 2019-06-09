#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/super4pcs.h>

#include <gr/shared.h>
#include "../demo-utils.h"

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
using namespace gr;

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr object_icp_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);

  // Get input object and scene
  if (argc < 4)
  {
    pcl::console::print_error ("Syntax is: %s scene.obj object.obj [PARAMS]\n", argv[0]);
    Demo::printParameterList();
    return (-1);
  }

  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile (argv[2], *object) < 0 ||
      pcl::io::loadPCDFile (argv[1], *scene) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (-1);
  }

  if(int c = Demo::getArgs(argc, argv) != 0)
    {
      Demo::printUsage(argc, argv);
      exit(std::max(c,0));
    }

  pcl::Super4PCS<PointNT,PointNT> align;
  Demo::setOptionsFromArgs(align.options_);

  // Downsample
//  pcl::console::print_highlight ("Downsampling...\n");
//  pcl::VoxelGrid<PointNT> grid;
//  const float leaf = 0.005f;r
//  grid.setLeafSize (leaf, leaf, leaf);
//  grid.setInputCloud (object);
//  grid.filter (*object);
//  grid.setInputCloud (scene);
//  grid.filter (*scene);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  align.setInputSource (object);
  align.setInputTarget (scene);

  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }

    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
//     Eigen::Matrix4f transformation=Eigen::Matrix4f::Identity();
//     transformation(0,0)=0.998;
//     transformation(0,1)=-0.055;
//     transformation(0,2)=-0.002;
//     transformation(1,0)=0.055;
//     transformation(1,1)=0.998 ;
//     transformation(1,2)=-0.035;
//     transformation(2,0)=0.004;
//     transformation(2,1)=0.035;
//     transformation(2,2)=0.999;
//     transformation(0,3)=-159.056;
//     transformation(1,3)=-246.621;
//     transformation(2,3)=-3.932;

    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    
//     pcl::NormalDistributionsTransform<PointNT, PointNT> aligner;
//     aligner.setTransformationEpsilon (0.001);
//     aligner.setStepSize (1);
//     aligner.setResolution (0.1);
//     aligner.setMaximumIterations (3500);
    
    pcl::IterativeClosestPoint<PointNT, PointNT> aligner;
    aligner.setMaxCorrespondenceDistance (10);
    aligner.setMaximumIterations (500);
    aligner.setTransformationEpsilon (1e-10);
    aligner.setEuclideanFitnessEpsilon (0.001);

    aligner.setInputSource (object);
    aligner.setInputTarget (scene);
    Eigen::Matrix4f init_guess = transformation;
    aligner.align (*object_icp_aligned, init_guess);
    pcl::transformPointCloud (*object, *object_icp_aligned, aligner.getFinalTransformation ());
    pcl::transformPointCloud (*object, *object_aligned, transformation);

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment - Super4PCS");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_icp_aligned, ColorHandlerT (object_icp_aligned, 255.0, 0.0, 255.0), "object_icp_aligned");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 255.0, 255.0, 255.0), "object_aligned");
    visu.spin ();

  return (0);
}
