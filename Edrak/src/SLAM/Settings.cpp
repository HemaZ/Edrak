#include "Edrak/SLAM/Settings.hpp"

namespace Edrak {
namespace SLAM {

std::optional<Settings> LoadSettings(const std::string filePath) {
  YAML::Node settingsFile;
  Settings loadedSettings;
  try {
    settingsFile = YAML::LoadFile(filePath);
  } catch (const YAML::BadFile &e) {
    return std::nullopt;
  }

  if (!settingsFile["Edrak"]["SLAM"]["Settings"])
    return std::nullopt;
  YAML::Node settingsNode = settingsFile["Edrak"]["SLAM"]["Settings"];

  // Number of features to keep.
  parseSetting<int>("nFeatures", loadedSettings.nFeatures, settingsNode);
  // Number of features to finish init.
  parseSetting<int>("nFeaturesToInit", loadedSettings.nFeaturesToInit,
                    settingsNode);
  // Number of features to assume good tracking.
  parseSetting<int>("nFeaturesTracking", loadedSettings.nFeaturesTracking,
                    settingsNode);
  // Number of features to switch to bad tracking.
  parseSetting<int>("nFeaturesBadTracking", loadedSettings.nFeaturesBadTracking,
                    settingsNode);
  // Number of features to add new keyframe.
  parseSetting<int>("nFeaturesNewKeyframe", loadedSettings.nFeaturesNewKeyframe,
                    settingsNode);
  // Number of iteration for current frame pose estimation.
  parseSetting<int>("nIterationsPoseEstimation",
                    loadedSettings.nIterationsPoseEstimation, settingsNode);
  // Chi2 (Mahalanobis distance) Threshold to consider the feature is outlier.
  parseSetting<double>("chi2Threshold", loadedSettings.chi2Threshold,
                       settingsNode);
  // Maximum number of iteration for g2o optimizer.
  parseSetting<int>("g2oOptimizerNIter", loadedSettings.g2oOptimizerNIter,
                    settingsNode);
  // Use the 5Pts Algorithm as an intial pose estimation.
  parseSetting<bool>("use5PtsAlgorithm", loadedSettings.use5PtsAlgorithm,
                     settingsNode);
  // Use Ceres as an intial pose estimation.
  parseSetting<bool>("useCeresOptimization",
                     loadedSettings.useCeresOptimization, settingsNode);

  return loadedSettings;
}
} // namespace SLAM

} // namespace Edrak
