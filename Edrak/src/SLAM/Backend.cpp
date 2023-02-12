#include "Edrak/SLAM/Backend.hpp"
#include "g2o_types.h"
namespace Edrak {

void Backend::UpdateMap() {
  if (map_ == nullptr) {
    logger_->info("Update map is called without setting the map");
    throw std::runtime_error("Map is not set. Can't update non existing map.");
  }
}

void Backend::RunBA() {
  auto kefyframes = map_->GetActiveKeyframes();
  auto landmarks = map_->GetActiveLandmarks();
  switch (solver_) {
  case Solver::CERES:
    CeresBA(kefyframes, landmarks);
    break;
  case Solver::G2O:
    G2oBA(kefyframes, landmarks);
    break;
  }
}

void Backend::CeresBA(const Map::KeyFramesData &keyframes,
                      const Map::LandmarksData &landmarks) {}

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
  //     if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) {
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
