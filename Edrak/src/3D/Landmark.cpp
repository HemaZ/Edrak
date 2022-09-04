#include "Edrak/3D/Landmark.hpp"
#include "Edrak/Images/Features.hpp"
namespace Edrak {

Landmark::Landmark(uint32_t id, const Eigen::Vector3d &pos)
    : id(id), position(pos) {}

Landmark::SharedPtr Landmark::CreateLandmark() {
  static uint32_t newLandmarkId = 0;
  Landmark::SharedPtr newPtr = std::make_shared<Landmark>();
  newPtr->id = newLandmarkId++;
  return newPtr;
}

void Landmark::RemoveObservation(
    std::shared_ptr<Edrak::Images::Features::Feature> feat) {
  std::lock_guard<std::mutex> lock(dataMutex);
  for (auto iter = observations.begin(); iter != observations.end(); ++iter) {
    if (iter->lock() == feat) {
      observations.erase(iter);
      feat->landmark.reset();
      nObservations--;
      break;
    }
  }
}
} // namespace Edrak
