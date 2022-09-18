#include "Edrak/SLAM/Frontend.hpp"

namespace Edrak {

bool Frontend::AddFrame(StereoFrame::SharedPtr frame) {
  currentFrame_ = frame;
  bool ret = false;
  switch (state_) {
  case FrontendState::INITIALIZING:
    ret = StereoInit();
    break;
  case FrontendState::TRACKING:
  case FrontendState::BAD_TRACKING:
    ret = Track();
    break;
  case FrontendState::LOST:
    ret = Reset();
    break;
  }
  prevFrame_ = currentFrame_;
  return ret;
}

bool Frontend::StereoInit() {
  using namespace Images::Features;
  if (currentFrame_ == nullptr) {
    logger_->error("Current Frame is nullptr !!");
    return false;
  }
  // Extract ORB features from left & right images.
  KeyPoints::KeyPoints kpsL, kpsR;
  Matches2D matches;
  Images::Features::ExtractORBMatches(
      currentFrame_->imgData, currentFrame_->rightImgData, kpsL, kpsR, matches);
  if (matches.size() < settings_.nFeaturesToInit) {
    logger_->info("Number of stereo frame features matching = {} is less than "
                  "nFeaturesToInit = {}",
                  matches.size(), settings_.nFeaturesToInit);
    return false;
  }

  return true;
}
} // namespace Edrak
