#ifndef __EDRAK_SLAM_SETTINGS_HPP__
#define __EDRAK_SLAM_SETTINGS_HPP__
#include "yaml-cpp/yaml.h"
#include <optional>
namespace Edrak {
namespace SLAM {
struct Settings {
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
  // Use the 5Pts Algorithm as an intial pose estimation.
  bool use5PtsAlgorithm = true;
  // Use Ceres as an intial pose estimation.
  bool useCeresOptimization = false;
};

template <typename T>
void parseSetting(std::string name, T &val, const YAML::Node &node) {
  if (node[name]) {
    val = node[name].as<T>();
  }
}
/**
 * @brief Load SLAM settings from a yaml file.
 *
 * @param filePath Yaml file path.
 * @return Settings loaded from the file. If couldn't read the file optional
 * none will be returned.
 */
std::optional<Settings> LoadSettings(const std::string filePath);
} // namespace SLAM

} // namespace Edrak

#endif