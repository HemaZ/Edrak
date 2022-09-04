#include "Edrak/SLAM/Map.hpp"
#include "Edrak/Images/Features.hpp"

namespace Edrak {
void Map::InsertKeyframe(Frame::SharedPtr frame) {
  currentFrame = frame;
  allKeyframes[frame->keyFrameId] = frame;
  activeKeyframes[frame->keyFrameId] = frame;
  if (activeKeyframes.size() > nActiveKeyframes) {
    RemoveOldKeyframe();
  }
}

void Map::RemoveOldKeyframe() {
  throw std::runtime_error("Not implemented yet");
}

void Map::CleanMap() { throw std::runtime_error("Not implemented yet"); }
} // namespace Edrak