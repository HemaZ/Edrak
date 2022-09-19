#ifndef __EDRAK_SLAM_FRONTEND_H__
#define __EDRAK_SLAM_FRONTEND_H__
#include "Edrak/Common/Logger.hpp"
#include "Edrak/Images/Calibration.hpp"
#include "Edrak/Images/Features.hpp"
#include "Edrak/Images/Frame.hpp"
#include "Edrak/Images/Triangulation.hpp"
#include "Edrak/SLAM/Map.hpp"
#include "Edrak/Visual/Viewer.hpp"
#include <opencv2/features2d.hpp>
namespace Edrak {
class Backend;
class Viewer;

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
  Frontend() {
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
  void SetCamera(const StereoCamera &stereoCamera) { camera_ = stereoCamera; }

private:
  bool Track();

  bool Reset();

  int TrackLastFrame();

  int EstimateCurrentPose();

  bool InsertKeyframe();

  /**
   * @brief Initialize SLAM with Stereo frame.
   * @return bool if initialization is successful.
   */
  bool StereoInit();

  bool BuildInitMap(const Images::Features::Matches2D &frameMatches,
                    const Images::Features::KeyPoints::KeyPoints &,
                    const Images::Features::KeyPoints::KeyPoints &);

  int TriangulateNewPoints();

  void SetObservationsForKeyFrame();

  StereoFrame::SharedPtr currentFrame_, prevFrame_;
  Map::SharedPtr map_;
  std::shared_ptr<Backend> backend_;
  std::shared_ptr<Viewer> viewer_;
  FrontendState state_ = FrontendState::INITIALIZING;
  StereoCamera camera_;
  Settings settings_;

  // Logger
  std::shared_ptr<spdlog::logger> logger_;
};

} // namespace Edrak

#endif // __EDRAK_SLAM_FRONTEND_H__