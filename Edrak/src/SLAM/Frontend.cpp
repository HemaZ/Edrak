#include "Edrak/SLAM/Frontend.hpp"
#include "Edrak/SLAM/Backend.hpp"
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
    logger_->error("Current frame is nullptr !!");
    return false;
  }
  // Extract ORB features from left & right images.
  KeyPoints::KeyPoints kpsL, kpsR;
  Matches2D matches;
  logger_->info("Extracting ORB features from left and right image");
  Images::Features::ExtractORBMatches(
      currentFrame_->imgData, currentFrame_->rightImgData, kpsL, kpsR, matches);
  logger_->info("Matched {} keypoints between left and right image.",
                matches.size());

  // Setting left image features.
  logger_->info("Setting the left image features.");
  for (size_t i = 0; i < kpsL.size(); i++) {
    auto feat =
        std::make_shared<Images::Features::Feature>(currentFrame_, kpsL[i]);
    currentFrame_->features.push_back(feat);
  }

  // Mark the features in the right image which are matched in left image
  std::vector<bool> matched(kpsR.size(), false);
  for (size_t i = 0; i < matches.size(); i++) {
    int kpLocation = matches[i].trainIdx;
    matched[kpLocation] = true;
  }
  // Set the right image features.
  logger_->info("Setting the right image features.");
  for (size_t i = 0; i < kpsR.size(); i++) {
    auto feat =
        std::make_shared<Images::Features::Feature>(currentFrame_, kpsR[i]);
    if (matched[i]) {
      feat->matchedOnLeftImg = true;
    }
    currentFrame_->featuresRight.push_back(feat);
  }

  if (matches.size() < settings_.nFeaturesToInit) {
    logger_->info("Number of stereo frame features matching = {} is less than "
                  "nFeaturesToInit = {}",
                  matches.size(), settings_.nFeaturesToInit);
    return false;
  }
  // Building inital map
  bool buildMap = BuildInitMap(matches, kpsL, kpsR);
  if (buildMap) {
    logger_->info("Building InitMap success. Tracking");
    state_ = FrontendState::TRACKING;
    if (viewer_) {
      logger_->info("submitting current frame and map to the viewer.");
      viewer_->AddCurrentFrame(currentFrame_);
      viewer_->UpdateMap();
    }
  } else {
    logger_->warn("Building initMap failed");
    return false;
  }
  return true;
}
bool Frontend::BuildInitMap(
    const Images::Features::Matches2D &matches,
    const Images::Features::KeyPoints::KeyPoints &leftKps,
    const Images::Features::KeyPoints::KeyPoints &rightKps) {
  std::vector<Sophus::SE3d> poses{camera_.leftCamera.pose,
                                  camera_.rightCamera.pose};
  size_t nLandmarks = 0;
  Eigen::Vector3d positionWorld = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < matches.size(); i++) {
    int lKpIdx = matches[i].queryIdx;
    int rKpIdx = matches[i].trainIdx;
    auto pointLeft = leftKps[lKpIdx].pt;
    auto pointRight = rightKps[rKpIdx].pt;
    bool triangSuccess =
        Images::Triangulation(poses, {pointLeft, pointRight}, positionWorld);
    if (triangSuccess && positionWorld[2] > 0) {
      auto newLandmark = Edrak::Landmark::CreateLandmark();
      newLandmark->Position(positionWorld);
      // Adding Observations to current landmark.
      newLandmark->AddObservation(currentFrame_->features[lKpIdx]);
      newLandmark->AddObservation(currentFrame_->featuresRight[rKpIdx]);

      currentFrame_->features[lKpIdx]->landmark = newLandmark;
      currentFrame_->featuresRight[rKpIdx]->landmark = newLandmark;
      nLandmarks++;
      map_->InsertLandmark(newLandmark);
    }
  }

  currentFrame_->isKeyFrame = true;
  map_->InsertKeyframe(currentFrame_);
  backend_->UpdateMap();

  logger_->info("Initial map created with {} landmarks.", nLandmarks);
  return true;
}
bool Frontend::Track() {}
bool Frontend::Reset() {}
} // namespace Edrak
