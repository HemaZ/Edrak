#include "Edrak/SLAM/Map.hpp"
#include "Edrak/Images/Features.hpp"
#include <limits>

namespace Edrak {
void Map::InsertKeyframe(StereoFrame::SharedPtr frame) {
  std::lock_guard<std::mutex> lock(dataMutex);
  currentFrame = frame;
  allKeyframes.insert({frame->keyFrameId, frame});
  activeKeyframes.insert({frame->keyFrameId, frame});
  if (activeKeyframes.size() > nActiveKeyframes) {
    RemoveOldKeyframe();
  }
}

void Map::InsertLandmark(Landmark::SharedPtr landmark) {
  std::lock_guard<std::mutex> lock(dataMutex);
  allLandmarks.insert({landmark->id, landmark});
  activeLandmarks.insert({landmark->id, landmark});
}

void Map::RemoveOldKeyframe() {
  if (currentFrame == nullptr) {
    return;
  }

  double minDistance = 0.0;
  double maxDistance = std::numeric_limits<double>::max();
  uint32_t minKfId, maxKfId;
  auto Twc = currentFrame->Twc();

  for (const auto &kf : activeKeyframes) {
    if (kf.second == currentFrame) {
      continue;
    }
    auto distance = (kf.second->Tcw() * Twc).log().norm();
    if (distance > maxDistance) {
      maxDistance = distance;
      maxKfId = kf.first;
    }
    if (distance < minDistance) {
      minDistance = distance;
      minKfId = kf.first;
    }
  }

  StereoFrame::SharedPtr frameToRemove = nullptr;
  if (minDistance < minDistanceFrame) {
    frameToRemove = activeKeyframes[minKfId];
  } else {
    frameToRemove = activeKeyframes[maxKfId];
  }

  // Remove the frame
  activeKeyframes.erase(frameToRemove->keyFrameId);

  for (auto feat : frameToRemove->features) {
    auto landmark = feat->landmark.lock();
    if (landmark) {
      landmark->RemoveObservation(feat);
    }
  }

  for (auto feat : frameToRemove->featuresRight) {
    auto landmark = feat->landmark.lock();
    if (landmark) {
      landmark->RemoveObservation(feat);
    }
  }
  CleanMap();
}

void Map::CleanMap() {
  int cnt_landmark_removed = 0;
  for (auto iter = activeLandmarks.begin(); iter != activeLandmarks.end();) {
    if (iter->second->nObservations == 0) {
      iter = activeLandmarks.erase(iter);
      cnt_landmark_removed++;
    } else {
      ++iter;
    }
  }
}
} // namespace Edrak