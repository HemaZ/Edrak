//
// Created by gaoxiang on 19-5-4.
//
#include "Edrak/SLAM/Viewer.hpp"
#include "Edrak/Images/Features.hpp"
#include "Edrak/Images/Frame.hpp"
#include <unistd.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>

namespace Edrak {

Viewer::Viewer() {
  viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
  viewer_running_ = false;
  viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Edrak::StereoFrame::SharedPtr current_frame) {
  std::unique_lock<std::mutex> lck(viewer_data_mutex_);
  current_frame_ = current_frame;
  trajectory_.push_back(current_frame->Twc());
}

void Viewer::UpdateMap() {
  std::unique_lock<std::mutex> lck(viewer_data_mutex_);
  assert(map_ != nullptr);
  active_keyframes_ = map_->GetActiveKeyframes();
  active_landmarks_ = map_->GetActiveLandmarks();
  map_updated_ = true;
}

void Viewer::ThreadLoop() {
  pangolin::CreateWindowAndBind("Edrak", 1366, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState vis_camera(
      pangolin::ProjectionMatrix(1366, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &vis_display =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(vis_camera));

  const float blue[3] = {0, 0, 1};
  const float green[3] = {0, 1, 0};
  const float yellow[3] = {1, 1, 0};

  while (!pangolin::ShouldQuit() && viewer_running_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    vis_display.Activate(vis_camera);

    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    if (current_frame_) {
      DrawFrame(current_frame_->Twc(), yellow);
      FollowCurrentFrame(vis_camera);

      cv::Mat img = PlotFrameImage();
      cv::namedWindow("image", cv::WINDOW_NORMAL);
      cv::imshow("image", img);
      cv::waitKey(1);
    }

    if (map_) {
      DrawMapPoints();
    }

    if (!trajectory_.empty()) {
      for (const auto &pose : trajectory_) {
        DrawFrame(pose, green);
      }
    }

    pangolin::FinishFrame();
    usleep(5000);
  }

  //   LOG(INFO) << "Stop viewer";
}

cv::Mat Viewer::PlotFrameImage() {
  cv::Mat img_out;
  cv::cvtColor(current_frame_->imgData, img_out, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < current_frame_->features.size(); ++i) {
    if (current_frame_->features[i]->landmark.lock()) {
      auto feat = current_frame_->features[i];
      cv::circle(img_out, feat->position.pt, 2, cv::Scalar(0, 0, 255), 2);
    }
  }
  return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera) {
  Sophus::SE3 Twc = current_frame_->Twc();
  pangolin::OpenGlMatrix m(Twc.matrix());
  vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(const Sophus::SE3d &Twc, const float *color) {
  // Sophus::SE3d Twc = frame->Twc();
  const float sz = 1.0;
  const int line_width = 2.0;
  const float fx = 721.5377;
  const float fy = 721.5377;
  const float cx = 609.5593;
  const float cy = 172.8540;
  const float width = 1.392000e+03;
  const float height = 5.120000e+02;

  glPushMatrix();

  Sophus::Matrix4f m = Twc.matrix().template cast<float>();
  glMultMatrixf((GLfloat *)m.data());

  if (color == nullptr) {
    glColor3f(1, 0, 0);
  } else
    glColor3f(color[0], color[1], color[2]);

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glEnd();
  glPopMatrix();
}

void Viewer::DrawMapPoints() {
  const float red[3] = {1.0, 0, 0};
  for (auto &kf : active_keyframes_) {
    DrawFrame(kf.second->Twc(), red);
  }

  glPointSize(2);
  glBegin(GL_POINTS);
  for (auto &landmark : active_landmarks_) {
    auto pos = landmark.second->Position();
    glColor3f(red[0], red[1], red[2]);
    glVertex3d(pos[0], pos[1], pos[2]);
  }
  glEnd();
}
void Viewer::AddCurrentTrajectory(const Edrak::TrajectoryD &traj) {
  std::unique_lock<std::mutex> lck(viewer_data_mutex_);
  trajectory_ = traj;
}

void Viewer::DrawTrajectory() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glLineWidth(2);
  for (size_t i = 0; i < trajectory_.size(); i++) {

    Eigen::Vector3d Ow = trajectory_[i].translation();
    Eigen::Vector3d Xw = trajectory_[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
    Eigen::Vector3d Yw = trajectory_[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
    Eigen::Vector3d Zw = trajectory_[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Xw[0], Xw[1], Xw[2]);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Yw[0], Yw[1], Yw[2]);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Zw[0], Zw[1], Zw[2]);
    glEnd();
  }
  for (size_t i = 0; i < trajectory_.size(); i++) {
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    auto p1 = trajectory_[i], p2 = trajectory_[i + 1];
    glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
    glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
    glEnd();
  }
  trajectory_.clear();
}

} // namespace Edrak
