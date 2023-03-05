#include "Edrak/SLAM/Backend.hpp"
#include "ceres_problems.hpp"
#include "g2o_types.h"
namespace Edrak {

void Backend::UpdateMap() {
  if (map_ == nullptr) {
    logger_->info("Update map is called without setting the map");
    throw std::runtime_error("Map is not set. Can't update non existing map.");
  }
  RunLocalBA();
}

void Backend::OptimizeMap() {
  if (map_ == nullptr) {
    logger_->info("Update map is called without setting the map");
    throw std::runtime_error("Map is not set. Can't update non existing map.");
  }
  RunGlobalBA();
}

void Backend::RunLocalBA() {
  auto kefyframes = map_->GetActiveKeyframes();
  auto landmarks = map_->GetActiveLandmarks();
  logger_->info("Running BA on {} Keyframe and {} map point", kefyframes.size(),
                landmarks.size());
  switch (solver_) {
  case Solver::CERES:
    CeresBA(kefyframes, landmarks);
    break;
  case Solver::G2O:
    G2oBA(kefyframes, landmarks);
    break;
  }
}

void Backend::RunGlobalBA() {
  auto kefyframes = map_->GetAllKeyframes();
  auto landmarks = map_->GetAllLandmarks();
  logger_->info("Running BA on {} Keyframe and {} map point", kefyframes.size(),
                landmarks.size());
  switch (solver_) {
  case Solver::CERES:
    CeresBA(kefyframes, landmarks);
    break;
  case Solver::G2O:
    G2oBA(kefyframes, landmarks);
    break;
  }
}

void Backend::CeresBA(Map::KeyFramesData &keyframes,
                      Map::LandmarksData &landmarks) {
  ceres::Problem problem;
  std::vector<std::vector<double>> kfsQuat;
  std::vector<std::vector<double>> kfsTrans;
  std::vector<std::shared_ptr<Landmark>> landmarksPtrs;
  // std::vector<std::vector<double>> landmarksPts;;
  std::unordered_map<uint32_t, std::vector<double>> landmarksPts;

  for (auto &kf : keyframes) {
    uint32_t keyFrameId = kf.first;
    Edrak::StereoFrame::SharedPtr frame = kf.second;
    Sophus::SE3d pose = frame->Tcw();
    Eigen::Vector3d translation = pose.translation();
    Eigen::Quaternion<double> quat(pose.so3().params());
    std::vector<double> quaterion = {quat.w(), quat.x(), quat.y(), quat.z()};
    std::vector<double> trans = {translation.x(), translation.y(),
                                 translation.z()};
    kfsQuat.push_back(quaterion);
    kfsTrans.push_back(trans);
    for (auto &feature : frame->features) {
      if (feature->isOutlier) {
        continue;
      }
      Edrak::Landmark::SharedPtr landmarkPtr = feature->landmark.lock();
      if (landmarkPtr == nullptr) {
        continue;
      }
      landmarksPtrs.push_back(landmarkPtr);
      auto landmarkPosition = landmarkPtr->Position();
      if (landmarksPts.count(landmarkPtr->id) == 0) {
        landmarksPts[landmarkPtr->id] = {
            landmarkPosition.x(), landmarkPosition.y(), landmarkPosition.z()};
      }
      ceres::CostFunction *cost_function;
      cost_function = ReprojectionErrorBA::Create(
          feature->position.pt.x, feature->position.pt.y,
          camera_.leftCamera.calibration);
      // If enabled use Huber's loss function.
      ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
      problem.AddResidualBlock(cost_function, loss_function,
                               kfsQuat.back().data(), kfsTrans.back().data(),
                               landmarksPts[landmarkPtr->id].data());
      ceres::Manifold *quaternion_manifold = new ceres::QuaternionManifold;
      problem.SetManifold(kfsQuat.back().data(), quaternion_manifold);
    }
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  /// TODO Set the optimized Kfs poses and landmarks positions
  /// to the map keyframes and landmarks
  size_t kfIdX = 0;
  for (auto &kf : keyframes) {
    auto translationEst = kfsTrans[kfIdX];
    auto quatEst = kfsQuat[kfIdX];
    Eigen::Vector3d translation(translationEst.data());
    Eigen::Quaternion<double> quatEstEigen(quatEst.data());
    Sophus::SE3d Tcw(quatEstEigen, translation);
    kf.second->Tcw(Tcw);
    ++kfIdX;
  }
  for (size_t landmarkIdx = 0; landmarkIdx < landmarksPtrs.size();
       landmarkIdx++) {
    auto landmark = landmarksPtrs[landmarkIdx];
    auto landmarkPt = landmarksPts[landmark->id];
    Eigen::Vector3d position(landmarkPt.data());
    landmark->Position(position);
  }
}

void Backend::G2oBA(const Map::KeyFramesData &keyframes,
                    const Map::LandmarksData &landmarks) {
  // setup g2o
  // typedef g2o::BlockSolver_6_3 BlockSolverType;
  // typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
  //     LinearSolverType;
  // auto solver = new g2o::OptimizationAlgorithmLevenberg(
  //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  // g2o::SparseOptimizer optimizer;
  // optimizer.setAlgorithm(solver);

  // // pose 顶点，使用Keyframe id
  // std::map<unsigned long, VertexPose *> vertices;
  // unsigned long max_kf_id = 0;
  // for (auto &keyframe : keyframes) {
  //   auto kf = keyframe.second;
  //   VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
  //   vertex_pose->setId(kf->keyFrameId);
  //   vertex_pose->setEstimate(kf->Tcw());
  //   optimizer.addVertex(vertex_pose);
  //   if (kf->keyFrameId > max_kf_id) {
  //     max_kf_id = kf->keyFrameId;
  //   }

  //   vertices.insert({kf->keyFrameId, vertex_pose});
  // }

  // // 路标顶点，使用路标id索引
  // std::map<unsigned long, VertexXYZ *> vertices_landmarks;

  // // K 和左右外参
  // Mat33 K = camera_.leftCamera.calibration.K();
  // SE3 left_ext = camera_.leftCamera.pose;
  // SE3 right_ext = camera_.rightCamera.pose;

  // // edges
  // int index = 1;
  // double chi2_th = 5.991; // robust kernel 阈值
  // std::map<EdgeProjection *, Images::Features::Feature::SharedPtr>
  //     edges_and_features;

  // for (auto &landmark : landmarks) {
  //   if (landmark.second->isOutlier)
  //     continue;
  //   unsigned long landmark_id = landmark.second->id;
  //   auto observations = landmark.second->GetObservations();
  //   for (auto &obs : observations) {
  //     if (obs.lock() == nullptr)
  //       continue;
  //     auto feat = obs.lock();
  //     if (feat->isOutlier || feat->frame.lock() == nullptr)
  //       continue;

  //     auto frame = feat->frame.lock();
  //     EdgeProjection *edge = nullptr;
  //     if (feat->matchedOnLeftImg) {
  //       edge = new EdgeProjection(K, left_ext);
  //     } else {
  //       edge = new EdgeProjection(K, right_ext);
  //     }

  //     // 如果landmark还没有被加入优化，则新加一个顶点
  //     if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
  //     {
  //       VertexXYZ *v = new VertexXYZ;
  //       v->setEstimate(landmark.second->Position());
  //       v->setId(landmark_id + max_kf_id + 1);
  //       v->setMarginalized(true);
  //       vertices_landmarks.insert({landmark_id, v});
  //       optimizer.addVertex(v);
  //     }

  //     edge->setId(index);
  //     edge->setVertex(0, vertices.at(frame->keyFrameId));     // pose
  //     edge->setVertex(1, vertices_landmarks.at(landmark_id)); // landmark
  //     edge->setMeasurement(Vec2(feat->position.pt.x, feat->position.pt.y));
  //     edge->setInformation(Mat22::Identity());
  //     auto rk = new g2o::RobustKernelHuber();
  //     rk->setDelta(chi2_th);
  //     edge->setRobustKernel(rk);
  //     edges_and_features.insert({edge, feat});

  //     optimizer.addEdge(edge);

  //     index++;
  //   }
  // }

  // // do optimization and eliminate the outliers
  // optimizer.initializeOptimization();
  // optimizer.optimize(10);

  // int cnt_outlier = 0, cnt_inlier = 0;
  // int iteration = 0;
  // while (iteration < 5) {
  //   cnt_outlier = 0;
  //   cnt_inlier = 0;
  //   // determine if we want to adjust the outlier threshold
  //   for (auto &ef : edges_and_features) {
  //     if (ef.first->chi2() > chi2_th) {
  //       cnt_outlier++;
  //     } else {
  //       cnt_inlier++;
  //     }
  //   }
  //   double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
  //   if (inlier_ratio > 0.5) {
  //     break;
  //   } else {
  //     chi2_th *= 2;
  //     iteration++;
  //   }
  // }

  // for (auto &ef : edges_and_features) {
  //   if (ef.first->chi2() > chi2_th) {
  //     ef.second->isOutlier = true;
  //     // remove the observation
  //     ef.second->landmark.lock()->RemoveObservation(ef.second);
  //   } else {
  //     ef.second->isOutlier = false;
  //   }
  // }

  // LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
  //           << cnt_inlier;

  // // Set pose and lanrmark position
  // for (auto &v : vertices) {
  //   keyframes.at(v.first)->Tcw(v.second->estimate());
  // }
  // for (auto &v : vertices_landmarks) {
  //   landmarks.at(v.first)->Position(v.second->estimate());
  // }
}
} // namespace Edrak
