#include "Edrak/SLAM/Frontend.hpp"
#include "Edrak/SLAM/Backend.hpp"
#include "ceres_problems.hpp"
#include "g2o_types.h"
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

bool Frontend::DetectFeatures(KeyPoints::KeyPoints &kpsL,
                              KeyPoints::KeyPoints &kpsR, Matches2D &matches) {
  // Extract ORB features from left & right images.
  logger_->info("Extracting ORB features from left and right image");
  Images::Features::ExtractORBMatches(
      currentFrame_->imgData, currentFrame_->rightImgData, kpsL, kpsR,
      currentFrame_->orbDescriptors, currentFrame_->rightImgDescriptors,
      matches);
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

  return true;
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
  DetectFeatures(kpsL, kpsR, matches);

  // Return false if number of ORB matches is very low
  if (matches.size() < settings_.nFeaturesToInit) {
    logger_->info("Number of stereo frame features matching = {} is less than "
                  "nFeaturesToInit = {}",
                  matches.size(), settings_.nFeaturesToInit);
    return false;
  }

  // Building inital map
  bool buildMap = BuildInitMap(matches, kpsL, kpsR);
  if (!buildMap) {
    logger_->warn("Building initMap failed");
    return false;
  }

  logger_->info("Building InitMap success. Tracking");
  state_ = FrontendState::TRACKING;
  if (viewer_) {
    logger_->info("submitting current frame and map to the viewer.");
    viewer_->AddCurrentFrame(currentFrame_);
    viewer_->UpdateMap();
  }

  return true;
}

bool Frontend::BuildInitMap(
    const Images::Features::Matches2D &matches,
    const Images::Features::KeyPoints::KeyPoints &leftKps,
    const Images::Features::KeyPoints::KeyPoints &rightKps) {
  std::vector<Sophus::SE3d> poses{camera_.leftCamera.pose,
                                  camera_.rightCamera.pose};
  if (!map_) {
    logger_->error("Map is not set, please SetMap before pushing first frame.");
    return false;
  }
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

bool Frontend::Track() {
  if (prevFrame_) {
    currentFrame_->Pose(relativeMotion_ * prevFrame_->Pose());
  }
  int nTrackedPointsLastFrame = TrackLastFrame();
  int nTrackedPoints = EstimateCurrentPoseCeres();
  if (nTrackedPoints > settings_.nFeaturesTracking) {
    state_ = FrontendState::TRACKING;
  } else if (nTrackedPoints > settings_.nFeaturesBadTracking) {
    state_ = FrontendState::BAD_TRACKING;
  } else {
    state_ = FrontendState::LOST;
  }

  if (nTrackedPoints < settings_.nFeaturesNewKeyframe) {
    InsertKeyframe();
  }
  relativeMotion_ = currentFrame_->Pose() * prevFrame_->Pose().inverse();
  if (viewer_) {
    viewer_->AddCurrentFrame(currentFrame_);
  }
  return true;
}

int Frontend::TrackLastFrame() {
  logger_->debug("Extracting current frame left images orb features");
  Images::Features::KeyPoints::KeyPoints currentKps;
  Images::Features::ORB(currentFrame_->imgData, currentKps,
                        currentFrame_->orbDescriptors);
  logger_->debug("Extracted {} features from current frame", currentKps.size());

  Edrak::Images::Features::Matches2D matches;
  Images::Features::FeaturesMatching(prevFrame_->orbDescriptors,
                                     currentFrame_->orbDescriptors, matches);
  int nTrackedPoints = matches.size();
  logger_->info("Number of tracked points {}", nTrackedPoints);
  for (const auto &match : matches) {
    int iCurrent = match.trainIdx;
    int iPrev = match.queryIdx;
    auto feat = std::make_shared<Images::Features::Feature>(
        currentFrame_, currentKps[iCurrent]);
    feat->landmark = prevFrame_->features[iPrev]->landmark;
    currentFrame_->features.push_back(feat);
  }
  return nTrackedPoints;
}

int Frontend::EstimateCurrentPoseCeres() {
  ceres::Problem problem;
  Sophus::SE3d pose = currentFrame_->Pose();
  Eigen::Vector3d translation = pose.translation();
  Eigen::Quaternion<double> quat(pose.so3().params());

  double T[] = {quat.w(),        quat.x(),        quat.y(),       quat.z(),
                translation.x(), translation.y(), translation.z()};
  for (size_t i = 0; i < currentFrame_->features.size(); i++) {
    // We get a shared_ptr of the weak ptr. Because the raw ptr could
    // have been destroyed from another thread.
    auto landmark = currentFrame_->features[i]->landmark.lock();
    if (!landmark) {
      // Continue if the feature is not associated with a landmark.
      continue;
    }
    double x = currentFrame_->features[i]->position.pt.x;
    double y = currentFrame_->features[i]->position.pt.y;
    Eigen::Vector3d position3D = landmark->Position();
    // We get a shared_ptr of the weak ptr. Because the raw ptr could
    // have been destroyed from another thread.
    ceres::CostFunction *cost_function = ReprojectionError::Create(
        x, y, camera_.leftCamera.calibration, position3D.data());
    problem.AddResidualBlock(cost_function, nullptr, T);
  }
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  // Sophus::SE3d new_pose = createSE3(T);
  currentFrame_->Pose(pose);
  return 0;
}

int Frontend::EstimateCurrentPose() {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // Create current frame location as vertex
  Edrak::VertexPose *vertexPose = new Edrak::VertexPose();
  vertexPose->setId(0);
  vertexPose->setEstimate(currentFrame_->Pose());
  optimizer.addVertex(vertexPose);

  // Get Left Camera Intrinsics Matrix K
  Eigen::Matrix3d K = camera_.leftCamera.calibration.K();

  // Create edges between current frame and landmarks.
  int edgeIdx = 1;
  std::vector<EdgeProjectionPoseOnly *> edges;
  std::vector<Images::Features::Feature::SharedPtr> features;
  for (size_t i = 0; i < currentFrame_->features.size(); i++) {
    // We get a shared_ptr of the weak ptr. Because the raw ptr could
    // have been destroyed from another thread.
    auto landmark = currentFrame_->features[i]->landmark.lock();
    if (!landmark) {
      // Continue if the feature is not associated with a landmark.
      continue;
    }
    features.push_back(currentFrame_->features[i]);
    // Create a new edge with landmark 3d position and Camera K matrix.
    // This edge represents the error between the pixel location (detected by
    // ORB) and the 3d-2d projection of the landmark on the camera plane.
    EdgeProjectionPoseOnly *edge =
        new EdgeProjectionPoseOnly(landmark->Position(), K);
    edge->setId(edgeIdx++);
    edge->setVertex(0, vertexPose);
    Eigen::Vector2d position(currentFrame_->features[i]->position.pt.x,
                             currentFrame_->features[i]->position.pt.y);
    edge->setMeasurement(position);
    edge->setInformation(Eigen::Matrix2d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    edges.push_back(edge);
    optimizer.addEdge(edge);
  }
  logger_->info("Pushed {} edges for optimization.", edges.size());

  // Run the optimizations for N Iterations
  int nOutliers = 0;
  for (size_t iteration = 0; iteration < settings_.nIterationsPoseEstimation;
       iteration++) {
    vertexPose->setEstimate(currentFrame_->Pose());
    optimizer.initializeOptimization();
    optimizer.optimize(settings_.g2oOptimizerNIter);
    nOutliers = 0;

    // Count number of outliers
    for (size_t i = 0; i < edges.size(); i++) {
      auto e = edges[i];
      if (features[i]->isOutlier) {
        e->computeError();
      }
      if (e->chi2() > settings_.chi2Threshold) {
        features[i]->isOutlier = true;
        e->setLevel(1);
        nOutliers++;
      } else {
        features[i]->isOutlier = false;
        e->setLevel(0);
      }

      if (iteration == settings_.nIterationsPoseEstimation / 2) {
        e->setRobustKernel(nullptr);
      }
    }
  }
  logger_->info("Outlier/Inlier in pose estimating: {}/{}", nOutliers,
                features.size() - nOutliers);
  // Set pose and outliers
  currentFrame_->Pose(vertexPose->estimate());
  logger_->info("Current Pose = {}", currentFrame_->Pose().matrix());
  for (auto &feat : features) {
    if (feat->isOutlier) {
      feat->landmark.reset();
      feat->isOutlier = false; // maybe we can still use it in future
    }
  }
  return features.size() - nOutliers;
}

bool Frontend::InsertKeyframe() {
  // TODO
  logger_->info("Setting Frame {} as a keyframe", currentFrame_->frameId);
  currentFrame_->isKeyFrame = true;
  map_->InsertKeyframe(currentFrame_);
  SetObservationsForKeyFrame();
  // Finding new features
  KeyPoints::KeyPoints kpsL, kpsR;
  Matches2D matches;
  DetectFeatures(kpsL, kpsR, matches);
  // Creating new Landmarks.
  TriangulateNewPoints();

  // Update Backend
  backend_->UpdateMap();

  // Update viewer with new keyframe
  if (viewer_)
    viewer_->UpdateMap();

  return true;
}

void Frontend::SetObservationsForKeyFrame() {
  for (const auto &feat : currentFrame_->features) {
    auto landmark = feat->landmark.lock();
    if (landmark) {
      landmark->AddObservation(feat);
    }
  }
}

int Frontend::TriangulateNewPoints() {}

bool Frontend::Reset() { return false; }
} // namespace Edrak
