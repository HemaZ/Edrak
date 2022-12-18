#ifndef __EDRAK_SLAM_FRONTEND_H__
#define __EDRAK_SLAM_FRONTEND_H__
#include "Edrak/Common/Logger.hpp"
#include "Edrak/Images/Calibration.hpp"
#include "Edrak/Images/Features.hpp"
#include "Edrak/Images/Frame.hpp"
#include "Edrak/Images/Triangulation.hpp"
#include "Edrak/SLAM/Map.hpp"
#include "Edrak/SLAM/Viewer.hpp"
#include "Edrak/VO/2D/PoseEstmation.hpp"
#include <opencv2/features2d.hpp>
namespace Edrak {
class Backend;
class Viewer;
using namespace Images::Features;

struct FrontendSettings {
  // Number of features to keep.
  int nFeatures = 200;
  // Number of features to finish init.
  int nFeaturesToInit = 100;
  // Number of features to assume good tracking.
  int nFeaturesTracking = 50;
  // Number of features to switch to bad tracking.
  int nFeaturesBadTracking = 20;
  // Number of features to add new keyframe.
  int nFeaturesNewKeyframe = 80;
  // Number of iteration for current frame pose estimation.
  int nIterationsPoseEstimation = 4;
  // Chi2 (Mahalanobis distance) Threshold to consider the feature is outlier.
  double chi2Threshold = 5.991;
  // Maximum number of iteration for g2o optimizer.
  int g2oOptimizerNIter = 10;
};

/**
 * @brief Frontend tracking state.
 */
enum class FrontendState { INITIALIZING, TRACKING, BAD_TRACKING, LOST };

class Frontend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using SharedPtr = std::shared_ptr<Frontend>;
  using Settings = FrontendSettings;
  /**
   * @brief Construct a new Frontend for SLAM
   */
  Frontend() : map_{std::make_shared<Map>()} {
    logger_ = spdlog::basic_logger_mt("slam_frontend_logger",
                                      "logs/slam/frontend.txt");
  }

  /**
   * @brief Add a new stereo frame for processing.
   * @param frame New input stereo frame.
   * @return bool If successful.
   */
  bool AddFrame(StereoFrame::SharedPtr frame);

  /**
   * @brief Set a new map and replace the current one.
   * @param map @sa Map.
   */
  void SetMap(Map::SharedPtr map) { map_ = map; }

  /**
   * @brief Set the Backend for the current Frontend.
   * @param backend @sa Backend.
   */
  void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

  /**
   * @brief Set the Viewer to visualize the SLAM.
   *
   * @param viewer
   */
  void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

  /**
   * @brief Get the State of the frontend.
   *
   * @return FrontendState Current frontend state.
   */
  FrontendState GetState() const { return state_; }

  /**
   * @brief Set the stereo camera calibration.
   * @param left Pinhole camera model represents the left camera.
   * @param right Pinhole camera model represents the right camera.
   */
  void SetCamera(const StereoCamera &stereoCamera) {
    camera_ = stereoCamera;
    voPtr = std::make_shared<VO::PoseEstmation>(camera_.leftCamera.calibration);
  }

private:
  /**
   * @brief Finds ORB features in left and tight image, Does ORB matching and
   * returns the matched features.
   *
   * @param kpsL Output Left image's keypoints.
   * @param kpsR Output Right image's keypoints.
   * @param matches Output ORB matches.
   * @return boolean if success.
   */
  bool DetectFeatures(KeyPoints::KeyPoints &kpsL, KeyPoints::KeyPoints &kpsR,
                      Matches2D &matches);
  bool Track();

  bool Reset();

  int TrackLastFrame();

  int EstimateCurrentPose();

  int EstimateCurrentPoseCeres();

  bool InsertKeyframe();

  void SetObservationsForKeyFrame();

  /**
   * @brief Initialize SLAM with Stereo frame.
   * @return bool if initialization is successful.
   */
  bool StereoInit();

  bool BuildInitMap(const Images::Features::Matches2D &frameMatches,
                    const Images::Features::KeyPoints::KeyPoints &,
                    const Images::Features::KeyPoints::KeyPoints &);

  int TriangulateNewPoints();

  StereoFrame::SharedPtr currentFrame_, prevFrame_;
  Map::SharedPtr map_;
  std::shared_ptr<Backend> backend_;
  std::shared_ptr<Viewer> viewer_;
  FrontendState state_ = FrontendState::INITIALIZING;
  Sophus::SE3d relativeMotion_;
  StereoCamera camera_;
  Settings settings_;

  // Logger
  std::shared_ptr<spdlog::logger> logger_;

  // VO 5 point algorithm
  std::shared_ptr<VO::PoseEstmation> voPtr;
};

} // namespace Edrak

#endif // __EDRAK_SLAM_FRONTEND_H__