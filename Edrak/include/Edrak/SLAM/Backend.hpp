#ifndef __EDRAK_SLAM_BACKEND_H__
#define __EDRAK_SLAM_BACKEND_H__
#include "Edrak/Common/Logger.hpp"
#include "Edrak/Images/Calibration.hpp"
#include "Edrak/Images/Features.hpp"
#include "Edrak/Images/Frame.hpp"
#include "Edrak/SLAM/Map.hpp"

namespace Edrak {
class Backend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using SharedPtr = std::shared_ptr<Backend>;

  Backend() {
    logger_ =
        spdlog::basic_logger_mt("slam_backend_logger", "logs/slam/backend.txt");
  }

  /**
   * Set map for the Backend.
   */
  void SetMap(Map::SharedPtr map) { map_ = map; }

  /**
   * Update the map and run bundle adjustment.
   */
  void UpdateMap();

  /**
   * @brief Run Global BA on the whole map.
   *
   */
  void OptimizeMap();

  /**
   * The non-liner optimzation solver to use.
   */
  enum struct Solver { CERES, G2O };

  /**
   * @brief Set the stereo camera calibration.
   * @param left Pinhole camera model represents the left camera.
   * @param right Pinhole camera model represents the right camera.
   */
  void SetCamera(const StereoCamera &stereoCamera) { camera_ = stereoCamera; }

private:
  /**
   * Run bundle adjusmtent on the active keyframes and landmarks.
   */
  void RunLocalBA();

  /**
   * Run bundle adjusmtent on the map points and the keyframes.
   */
  void RunGlobalBA();

  /**
   * Run bundle adjustment using Ceres.
   */
  void CeresBA(Map::KeyFramesData &keyframes, Map::LandmarksData &landmarks)
      __attribute__((optimize(0)));

  /**
   * Run bundle adjeusmtent using G2O.
   */
  void G2oBA(const Map::KeyFramesData &keyframes,
             const Map::LandmarksData &landmarks);

  Solver solver_ = Solver::CERES;

  Map::SharedPtr map_;

  StereoCamera camera_;

  // Logger
  std::shared_ptr<spdlog::logger> logger_;
};

} // namespace Edrak

#endif