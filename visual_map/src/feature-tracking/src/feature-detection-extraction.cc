#include <array>

#include "feature-tracking/feature-detection-extraction.h"

#include <aslam/common/statistics/statistics.h>
#include <aslam/common/timer.h>
#include <aslam/frames/visual-frame.h>
//#include <aslam/tracker/tracking-helpers.h>
#include <gflags/gflags.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "feature-tracking/grided-detector.h"

DEFINE_bool(use_grided_detections, true, "Use multiple detectors on a grid?");

namespace feature_tracking {

FeatureDetectorExtractor::FeatureDetectorExtractor(const aslam::Camera& camera)
    : camera_(camera) {
  initialize();
}

void FeatureDetectorExtractor::initialize() {
  CHECK_LT(
      2 * detector_settings_.min_tracking_distance_to_image_border_px,
      camera_.imageWidth());
  CHECK_LT(
      2 * detector_settings_.min_tracking_distance_to_image_border_px,
      camera_.imageHeight());

  // No distance to the edges is required for the grided detector.
  const size_t orb_detector_edge_threshold =
      FLAGS_use_grided_detections
          ? 0u
          : detector_settings_.orb_detector_edge_threshold;

  detector_ = cv::ORB::create(
      detector_settings_.orb_detector_number_features,
      detector_settings_.orb_detector_scale_factor,
      detector_settings_.orb_detector_pyramid_levels,
      orb_detector_edge_threshold, detector_settings_.orb_detector_first_level,
      detector_settings_.orb_detector_WTA_K,
      detector_settings_.orb_detector_score_type,
      detector_settings_.orb_detector_patch_size,
      detector_settings_.orb_detector_fast_threshold);

  switch (extractor_settings_.descriptor_type) {
    case SweFeatureTrackingExtractorSettings::DescriptorType::kOcvFreak:
      extractor_ = cv::xfeatures2d::FREAK::create(
          extractor_settings_.rotation_invariant,
          extractor_settings_.scale_invariant,
          extractor_settings_.freak_pattern_scale,
          detector_settings_.orb_detector_pyramid_levels);
      break;
    default:
      LOG(FATAL) << "Unknown descriptor type.";
      break;
  }
}

cv::Ptr<cv::DescriptorExtractor> FeatureDetectorExtractor::getExtractorPtr()
    const {
  return extractor_;
}

void insertCvKeypointsAndDescriptorsIntoEmptyVisualFrame(
    const std::vector<cv::KeyPoint>& new_cv_keypoints, const cv::Mat& new_cv_descriptors,
    const double fixed_keypoint_uncertainty_px, aslam::VisualFrame* frame) {
  CHECK_NOTNULL(frame);
  CHECK(!frame->hasKeypointMeasurements() || frame->getNumKeypointMeasurements() == 0u);
  CHECK(!frame->hasDescriptors() || frame->getDescriptors().cols() == 0);
  CHECK(!frame->hasKeypointMeasurementUncertainties() ||
        frame->getKeypointMeasurementUncertainties().rows() == 0);
  CHECK(!frame->hasKeypointOrientations() || frame->getKeypointOrientations().rows() == 0);
  CHECK(!frame->hasKeypointScales() || frame->getKeypointScales().rows() == 0);
  CHECK(!frame->hasKeypointScores() || frame->getKeypointScores().rows() == 0);
  CHECK(!frame->hasTrackIds() || frame->getTrackIds().rows() == 0);
  CHECK_GT(fixed_keypoint_uncertainty_px, 0.0);
  CHECK_EQ(new_cv_keypoints.size(), static_cast<size_t>(new_cv_descriptors.rows));
  CHECK_EQ(new_cv_descriptors.type(), CV_8UC1);
  CHECK(new_cv_descriptors.isContinuous());

  const size_t kNumNewKeypoints = new_cv_keypoints.size();

  Eigen::Matrix2Xd new_keypoints_measurements;
  Eigen::VectorXd new_keypoint_scores;
  Eigen::VectorXd new_keypoint_scales;
  Eigen::VectorXd new_keypoint_orientations;
  new_keypoints_measurements.resize(Eigen::NoChange, kNumNewKeypoints);
  new_keypoint_scores.resize(kNumNewKeypoints);
  new_keypoint_scales.resize(kNumNewKeypoints);
  new_keypoint_orientations.resize(kNumNewKeypoints);
  for (size_t idx = 0u; idx < kNumNewKeypoints; ++idx) {
    const cv::KeyPoint& cv_keypoint = new_cv_keypoints[idx];
    new_keypoints_measurements.col(idx)(0) = static_cast<double>(cv_keypoint.pt.x);
    new_keypoints_measurements.col(idx)(1) = static_cast<double>(cv_keypoint.pt.y);
    new_keypoint_scores(idx) = static_cast<double>(cv_keypoint.response);
    new_keypoint_scales(idx) = static_cast<double>(cv_keypoint.size);
    new_keypoint_orientations(idx) = static_cast<double>(cv_keypoint.angle);
  }

  frame->swapKeypointMeasurements(&new_keypoints_measurements);
  frame->swapKeypointScores(&new_keypoint_scores);
  frame->swapKeypointScales(&new_keypoint_scales);
  frame->swapKeypointOrientations(&new_keypoint_orientations);

  Eigen::VectorXd uncertainties(kNumNewKeypoints);
  uncertainties.setConstant(fixed_keypoint_uncertainty_px);
  frame->swapKeypointMeasurementUncertainties(&uncertainties);

  // Set invalid track ids.
  Eigen::VectorXi track_ids(kNumNewKeypoints);
  track_ids.setConstant(-1);
  frame->swapTrackIds(&track_ids);

  frame->setDescriptors(
      // Switch cols/rows as Eigen is col-major and cv::Mat is row-major.
      Eigen::Map<aslam::VisualFrame::DescriptorsT>(
          new_cv_descriptors.data, new_cv_descriptors.cols, new_cv_descriptors.rows));
}

void FeatureDetectorExtractor::detectAndExtractFeatures(
    aslam::VisualFrame* frame) const {
  CHECK_NOTNULL(frame);
  CHECK(frame->hasRawImage())
      << "Can only detect keypoints if the frame has a raw image";
  CHECK_EQ(
      camera_.getId(),
      CHECK_NOTNULL(frame->getCameraGeometry().get())->getId());

  timing::Timer timer_detection("keypoint detection");

  std::vector<cv::KeyPoint> keypoints_cv;
  const cv::Mat& image = frame->getRawImage();

  if (FLAGS_use_grided_detections) {
    // Grided detection to ensure a certain distribution of keypoints across
    // the image.
    constexpr size_t kNumGridCols = 3;
    constexpr size_t kNumGridRows = 2;
    detectKeypointsGrided(
        detector_, image, /*detection_mask=*/cv::Mat(),
        detector_settings_.max_feature_count,
        detector_settings_.detector_nonmaxsuppression_radius,
        detector_settings_.detector_nonmaxsuppression_ratio_threshold,
        kNumGridCols, kNumGridRows, &keypoints_cv);
  } else {
    detector_->detect(image, keypoints_cv);

    if (detector_settings_.detector_use_nonmaxsuppression) {
      timing::Timer timer_nms("non-maximum suppression");
      localNonMaximumSuppression(
          camera_.imageHeight(),
          detector_settings_.detector_nonmaxsuppression_radius,
          detector_settings_.detector_nonmaxsuppression_ratio_threshold,
          &keypoints_cv);

      statistics::StatsCollector stat_nms(
          "non-maximum suppression (1 image) in ms");
      stat_nms.AddSample(timer_nms.Stop() * 1000);
    }
    cv::KeyPointsFilter::retainBest(
        keypoints_cv, detector_settings_.max_feature_count);
  }

  // The ORB detector tries to always return a constant number of keypoints.
  // If we get into an environment with very few good keypoint candidates
  // the detector adapts it's score such that it even detects keypoints that
  // are dust on the camera. Therefore, we put a lower bound on the threshold.
  size_t num_removed_keypoints = keypoints_cv.size();

  std::vector<cv::KeyPoint>::iterator it_erase_from = std::remove_if(
      keypoints_cv.begin(), keypoints_cv.end(), [this](const cv::KeyPoint& kp) {
        return kp.response <= detector_settings_.orb_detector_score_lower_bound;
      });
  keypoints_cv.erase(it_erase_from, keypoints_cv.end());

  num_removed_keypoints -= keypoints_cv.size();
  VLOG(4) << "Number of removed low score keypoints: " << num_removed_keypoints;

  if (extractor_settings_.flip_descriptor) {
    for (cv::KeyPoint& keypoint : keypoints_cv) {
      keypoint.angle = 180.0f;
    }
  } else {
    // We are doing this because:
    // - Stefan Leutenegger's BRISK implementation uses orientation information
    //   (in case of enabled rotational invariance) of OpenCV keypoints if the
    //   angles are not -1 and the ORB detector assigns orientations to
    //   keypoints.
    //   I don't know which orientation assignment is more robust so feel free
    //   to remove this if you know better. I think there is no significant
    //   performance difference.
    // - Also remove this if you use descriptors (e.g. OpenCV BRISK or ORB)
    //   that rely on orientation information provided by some detectors
    //   (e.g. ORB detector).
    for (cv::KeyPoint& keypoint : keypoints_cv) {
      keypoint.angle = -1.0f;
    }
  }
  timer_detection.Stop();

  timing::Timer timer_extraction("descriptor extraction");

  // Compute the descriptors.
  cv::Mat descriptors_cv;
  if (!keypoints_cv.empty()) {
    extractor_->compute(frame->getRawImage(), keypoints_cv, descriptors_cv);
    
  } else {
    descriptors_cv = cv::Mat(0, 0, CV_8UC1);
  }
  

  timer_extraction.Stop();

  // Note: It is important that the values are set even if there are no
  // keypoints as downstream code may rely on the keypoints being set.
  insertCvKeypointsAndDescriptorsIntoEmptyVisualFrame(
      keypoints_cv, descriptors_cv, detector_settings_.keypoint_uncertainty_px,
      frame);
}
}  // namespace feature_tracking
