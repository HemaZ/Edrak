#ifndef __EDRAK_SLAM_MAP_H__
#define __EDRAK_SLAM_MAP_H__

#include "Edrak/3D/Landmark.hpp"
#include "Edrak/Images/Features.hpp"
#include "Edrak/Images/Frame.hpp"
#include <memory>
#include <mutex>
#include <unordered_map>
namespace Edrak {
class Map {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using SharedPtr = std::shared_ptr<Map>;
  using LandmarksData =
      std::unordered_map<uint32_t, Edrak::Landmark::SharedPtr>;
  using KeyFramesData = std::unordered_map<uint32_t, Edrak::Frame::SharedPtr>;
  Map() {}
  /**
   * @brief
   *
   * @param frame
   */
  void InsertKeyframe(Frame::SharedPtr frame);

  /**
   * @brief
   *
   * @param landmark
   */
  void InsertLandmark(Landmark::SharedPtr landmark);

  /**
   * @brief Get the All Landmarks object
   *
   * @return LandmarksData
   */
  LandmarksData GetAllLandmarks() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return allLandmarks;
  }

  /**
   * @brief Get the Active Landmarks object
   *
   * @return LandmarksData
   */
  LandmarksData GetActiveLandmarks() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return activeLandmarks;
  }

  /**
   * @brief Get the All Keyframes object
   *
   * @return KeyFramesData
   */
  KeyFramesData GetAllKeyframes() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return allKeyframes;
  }

  /**
   * @brief Get the Active Keyframes object
   *
   * @return KeyFramesData
   */
  KeyFramesData GetActiveKeyframes() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return activeKeyframes;
  }

  /**
   * @brief
   *
   */
  void CleanMap();

private:
  std::mutex dataMutex;
  LandmarksData allLandmarks;
  LandmarksData activeLandmarks;
  KeyFramesData allKeyframes;
  KeyFramesData activeKeyframes;
  Frame::SharedPtr currentFrame;
  uint32_t nActiveKeyframes = 7;

  void RemoveOldKeyframe();
};

} // namespace Edrak

#endif // __MAP_H__