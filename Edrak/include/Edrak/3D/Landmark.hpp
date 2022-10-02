#ifndef __EDRAK_3D_LANDMARK_H__
#define __EDRAK_3D_LANDMARK_H__
#include <Eigen/Dense>
#include <list>
#include <memory>
#include <mutex>
namespace Edrak {
namespace Images {
namespace Features {
struct Feature;
}
} // namespace Images
struct Landmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SharedPtr = std::shared_ptr<Landmark>;
  uint32_t id = 0;
  bool isOutlier = false;
  // Position in 3D world.
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  // Position mutex.
  std::mutex dataMutex;
  // Number of observation times.
  uint32_t nObservations = 0;
  // Current observations to this landmark.
  std::list<std::weak_ptr<Edrak::Images::Features::Feature>> observations;

  Landmark() {}
  Landmark(uint32_t id, const Eigen::Vector3d &position);
  Eigen::Vector3d Position() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return position;
  }
  /**
   * @brief
   *
   * @param newPosition
   */
  void Position(const Eigen::Vector3d &newPosition) {
    std::lock_guard<std::mutex> lock(dataMutex);
    position = newPosition;
  }

  /**
   * @brief
   *
   * @param feature
   */
  void
  AddObservation(std::shared_ptr<Edrak::Images::Features::Feature> feature) {
    std::unique_lock<std::mutex> lock(dataMutex);
    observations.push_back(feature);
    nObservations++;
  }

  /**
   * @brief
   *
   * @param feat
   */
  void
  RemoveObservation(std::shared_ptr<Edrak::Images::Features::Feature> feat);

  /**
   * @brief Get the Obs object
   *
   * @return std::list<std::weak_ptr<Feature>>
   */
  std::list<std::weak_ptr<Edrak::Images::Features::Feature>> GetObservations() {
    std::unique_lock<std::mutex> lock(dataMutex);
    return observations;
  }

  // factory function
  static Landmark::SharedPtr CreateLandmark();
};

} // namespace Edrak

#endif // __EDRAK_3D_LANDMARK_H__