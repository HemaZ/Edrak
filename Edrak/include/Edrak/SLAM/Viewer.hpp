//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <pangolin/pangolin.h>
#include <thread>

#include "Edrak/Images/Frame.hpp"
#include "Map.hpp"

namespace Edrak {

/**
 * 可视化
 */
class Viewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Viewer> SharedPtr;

  Viewer();

  void SetMap(Map::SharedPtr map) { map_ = map; }

  void Close();

  // 增加一个当前帧
  void AddCurrentFrame(Edrak::StereoFrame::SharedPtr current_frame);

  // 更新地图
  void UpdateMap();

private:
  void ThreadLoop();

  void DrawFrame(Edrak::StereoFrame::SharedPtr frame, const float *color);

  void DrawMapPoints();

  void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

  /// plot the features in current frame into an image
  cv::Mat PlotFrameImage();

  Edrak::StereoFrame::SharedPtr current_frame_ = nullptr;
  Map::SharedPtr map_ = nullptr;

  std::thread viewer_thread_;
  bool viewer_running_ = true;

  std::unordered_map<uint32_t, Edrak::StereoFrame::SharedPtr> active_keyframes_;
  std::unordered_map<uint32_t, Edrak::Landmark::SharedPtr> active_landmarks_;
  bool map_updated_ = false;

  std::mutex viewer_data_mutex_;
};
} // namespace Edrak

#endif // MYSLAM_VIEWER_H
