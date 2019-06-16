#include "segmatch/rviz_utilities.hpp"
#include <laser_slam/common.hpp>
#include "segmatch/normal_estimators/normal_estimator.hpp"
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>
#include "segmatch/database.hpp"
#include "segmatch/segmatch.hpp"
#include <segmatch/utilities.hpp>
#include "ros/ros.h"
#include "segmatch/common.hpp"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

extern template class segmatch::DynamicVoxelGrid<segmatch::PclPoint, segmatch::MapPoint>;

segmatch::SegMatchParams getSegMatchParams(const ros::NodeHandle& nh, const std::string& prefix) {
    segmatch::SegMatchParams params;

    std::string ns = prefix + "/SegMatch";

    nh.getParam(ns + "/segmentation_radius_m",
                params.segmentation_radius_m);
    nh.getParam(ns + "/segmentation_height_above_m",
                params.segmentation_height_above_m);
    nh.getParam(ns + "/segmentation_height_below_m",
                params.segmentation_height_below_m);

    nh.getParam(ns + "/filter_boundary_segments",
                params.filter_boundary_segments);
    nh.getParam(ns + "/boundary_radius_m",
                params.boundary_radius_m);
    nh.getParam(ns + "/filter_duplicate_segments",
                params.filter_duplicate_segments);
    nh.getParam(ns + "/centroid_distance_threshold_m",
                params.centroid_distance_threshold_m);
    int min_time_between_segment_for_matches_s;
    nh.getParam(ns + "/min_time_between_segment_for_matches_s",
                min_time_between_segment_for_matches_s);
    params.min_time_between_segment_for_matches_ns =
        laser_slam::Time(min_time_between_segment_for_matches_s) * 1000000000u;
    nh.getParam(ns + "/check_pose_lies_below_segments",
                params.check_pose_lies_below_segments);
    nh.getParam(ns + "/radius_for_normal_estimation_m",
                params.radius_for_normal_estimation_m);
    nh.getParam(ns + "/normal_estimator_type",
                params.normal_estimator_type);

    // Local map parameters.
    nh.getParam(ns + "/LocalMap/voxel_size_m",
                params.local_map_params.voxel_size_m);
    nh.getParam(ns + "/LocalMap/min_points_per_voxel",
                params.local_map_params.min_points_per_voxel);
    nh.getParam(ns + "/LocalMap/radius_m",
                params.local_map_params.radius_m);
    nh.getParam(ns + "/LocalMap/min_vertical_distance_m",
                params.local_map_params.min_vertical_distance_m);
    nh.getParam(ns + "/LocalMap/max_vertical_distance_m",
                params.local_map_params.max_vertical_distance_m);
    nh.getParam(ns + "/LocalMap/neighbors_provider_type",
                params.local_map_params.neighbors_provider_type);

    // Descriptors parameters.
    nh.getParam(ns + "/Descriptors/descriptor_types",
                params.descriptors_params.descriptor_types);
    nh.getParam(ns + "/Descriptors/fast_point_feature_histograms_search_radius",
                params.descriptors_params.fast_point_feature_histograms_search_radius);
    nh.getParam(ns + "/Descriptors/fast_point_feature_histograms_normals_search_radius",
                params.descriptors_params.
                fast_point_feature_histograms_normals_search_radius);
    nh.getParam(ns + "/Descriptors/point_feature_histograms_search_radius",
                params.descriptors_params.point_feature_histograms_search_radius);
    nh.getParam(ns + "/Descriptors/point_feature_histograms_normals_search_radius",
                params.descriptors_params.point_feature_histograms_normals_search_radius);
    nh.getParam(ns + "/Descriptors/cnn_model_path",
                params.descriptors_params.cnn_model_path);
    nh.getParam(ns + "/Descriptors/semantics_nn_path",
                params.descriptors_params.semantics_nn_path);

    // Segmenter parameters.
    nh.getParam(ns + "/Segmenters/segmenter_type",
                params.segmenter_params.segmenter_type);
    nh.getParam(ns + "/Segmenters/min_cluster_size",
                params.segmenter_params.min_cluster_size);
    nh.getParam(ns + "/Segmenters/max_cluster_size",
                params.segmenter_params.max_cluster_size);
    nh.getParam(ns + "/Segmenters/radius_for_growing",
                params.segmenter_params.radius_for_growing);
    nh.getParam(ns + "/Segmenters/sc_smoothness_threshold_deg",
                params.segmenter_params.sc_smoothness_threshold_deg);
    nh.getParam(ns + "/Segmenters/sc_curvature_threshold",
                params.segmenter_params.sc_curvature_threshold);

    // Classifier parameters.
    nh.getParam(ns + "/Classifier/classifier_filename",
                params.classifier_params.classifier_filename);
    nh.getParam(ns + "/Classifier/threshold_to_accept_match",
                params.classifier_params.threshold_to_accept_match);

    nh.getParam(ns + "/Classifier/rf_max_depth",
                params.classifier_params.rf_max_depth);
    nh.getParam(ns + "/Classifier/rf_min_sample_ratio",
                params.classifier_params.rf_min_sample_ratio);
    nh.getParam(ns + "/Classifier/rf_regression_accuracy",
                params.classifier_params.rf_regression_accuracy);
    nh.getParam(ns + "/Classifier/rf_use_surrogates",
                params.classifier_params.rf_use_surrogates);
    nh.getParam(ns + "/Classifier/rf_max_categories",
                params.classifier_params.rf_max_categories);
    nh.getParam(ns + "/Classifier/rf_priors",
                params.classifier_params.rf_priors);
    nh.getParam(ns + "/Classifier/rf_calc_var_importance",
                params.classifier_params.rf_calc_var_importance);
    nh.getParam(ns + "/Classifier/rf_n_active_vars",
                params.classifier_params.rf_n_active_vars);
    nh.getParam(ns + "/Classifier/rf_max_num_of_trees",
                params.classifier_params.rf_max_num_of_trees);
    nh.getParam(ns + "/Classifier/rf_accuracy",
                params.classifier_params.rf_accuracy);

    nh.getParam(ns + "/Classifier/do_not_use_cars",
                params.classifier_params.do_not_use_cars);

    // Convenience copy to find the correct feature distance according to
    // descriptors types.
    nh.getParam(ns + "/Descriptors/descriptor_types",
                params.classifier_params.descriptor_types);

    nh.getParam(ns + "/Classifier/n_nearest_neighbours",
                params.classifier_params.n_nearest_neighbours);
    nh.getParam(ns + "/Classifier/enable_two_stage_retrieval",
                params.classifier_params.enable_two_stage_retrieval);
    nh.getParam(ns + "/Classifier/knn_feature_dim",
                params.classifier_params.knn_feature_dim);
    nh.getParam(ns + "/Classifier/apply_hard_threshold_on_feature_distance",
                params.classifier_params.apply_hard_threshold_on_feature_distance);
    nh.getParam(ns + "/Classifier/feature_distance_threshold",
                params.classifier_params.feature_distance_threshold);

    nh.getParam(ns + "/Classifier/normalize_eigen_for_knn",
                params.classifier_params.normalize_eigen_for_knn);
    nh.getParam(ns + "/Classifier/normalize_eigen_for_hard_threshold",
                params.classifier_params.normalize_eigen_for_hard_threshold);
    nh.getParam(ns + "/Classifier/max_eigen_features_values",
                params.classifier_params.max_eigen_features_values);


    // Geometric Consistency Parameters.
    nh.getParam(ns + "/GeometricConsistency/recognizer_type",
                params.geometric_consistency_params.recognizer_type);
    nh.getParam(ns + "/GeometricConsistency/resolution",
                params.geometric_consistency_params.resolution);
    nh.getParam(ns + "/GeometricConsistency/min_cluster_size",
                params.geometric_consistency_params.min_cluster_size);
    nh.getParam(ns + "/GeometricConsistency/max_consistency_distance_for_caching",
                params.geometric_consistency_params.max_consistency_distance_for_caching);

    return params;
}

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    visualization::publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}

std::shared_ptr<segmatch::SegMatch> segmatch_;

void show_target_pc(){
    segmatch::PointICloud target_representation;
    segmatch_->getTargetRepresentation(&target_representation);
    Eigen::Matrix3Xf points;
    Eigen::VectorXf intensities;
    points.resize(3,target_representation.points.size());
    intensities.resize(target_representation.points.size(),1);
    for(int i=0; i<target_representation.points.size(); i++){
        pcl::PointXYZI& target_segment_centroid = target_representation.points[i];
        points(0,i)=target_segment_centroid.x;
        points(1,i)=target_segment_centroid.y;
        points(2,i)=target_segment_centroid.z;
        intensities(i)=target_segment_centroid.intensity;
    }  
    visualization::publish3DPointsAsPointCloud(points, intensities, "map","target_mp");
}

void show_source_pc(Eigen::Matrix4f& pose_out){
    segmatch::PointICloud source_representation;
    segmatch_->getSourceRepresentation(&source_representation);
    std::cout<<"source pc count: "<<source_representation.points.size()<<std::endl;
    Eigen::Matrix3Xf points;
    Eigen::VectorXf intensities;
    points.resize(3,source_representation.points.size());
    intensities.resize(source_representation.points.size(),1);
    for(int i=0; i<source_representation.points.size(); i++){
        pcl::PointXYZI& segment_centroid = source_representation.points[i];
        
        Eigen::Vector4f posi_t;
        posi_t[0] = segment_centroid.x;
        posi_t[1] = segment_centroid.y;
        posi_t[2] = segment_centroid.z;
        posi_t[3] = 1.0;
        posi_t=pose_out*posi_t;
        
        points(0,i)=posi_t(0);
        points(1,i)=posi_t(1);
        points(2,i)=posi_t(2);
        intensities(i)=segment_centroid.intensity;
    }  
    visualization::publish3DPointsAsPointCloud(points, intensities, "map","source_mp");
}

int main(int argc, char **argv) {
    //typedef PointMatcher<float> PM;
    //typedef PM::DataPoints DP;
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "desc_matcher");
    ros::NodeHandle node_handle;
    std::string res_root=argv[1];
    std::string mode=argv[2];
    std::string semi_pcd=res_root+"/semi_pc.pcd";
    std::string desc_db_folder=res_root+"/pc_desc_database";
    
    segmatch_.reset(new segmatch::SegMatch());
    segmatch::SegMatchParams params=getSegMatchParams(node_handle, "/desc_matcher");
    segmatch_->init(params, 1);
    if(mode=="gen_map"){
        segmatch::MapCloud target_cloud;
        segmatch::loadCloud(semi_pcd, &target_cloud);
        std::cout<<target_cloud.points.size()<<std::endl;
        segmatch_->processAndSetAsTargetCloud(target_cloud);
        segmatch::database::exportSessionDataToDatabase(segmatch_->segmented_target_cloud_, desc_db_folder);
        show_target_pc();
        ros::spin();
    }else{
        int track_id=0;
        segmatch::MapCloud source_cloud;
        segmatch::loadCloud(semi_pcd, &source_cloud);
        std::cout<<source_cloud.points.size()<<std::endl;
        segmatch::database::importSessionDataFromDatabase(&segmatch_->segmented_target_cloud_, desc_db_folder);
        segmatch_->setAsTargetCloud();
        show_target_pc();
        segmatch_->segmented_source_clouds_[track_id] = segmatch::SegmentedCloud();
        segmatch_->processCloud(source_cloud,&segmatch_->segmented_source_clouds_[track_id]);
        segmatch::PairwiseMatches predicted_matches = segmatch_->findMatches();
        std::cout<<"predicted_matches: "<<predicted_matches.size()<<std::endl;
        Eigen::Matrix4f pose_out;
        segmatch::PairwiseMatches recognized_matches = segmatch_->recognize(predicted_matches, pose_out);
        std::cout<<"recognized_matches: "<<recognized_matches.size()<<std::endl;
        std::cout<<pose_out<<std::endl;
        show_source_pc(pose_out);
        ros::spin();
    }
    
    return 0;
}

