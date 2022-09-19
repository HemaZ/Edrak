#ifndef __EDRAK_VISUAL_VIEWER_HPP__
#define __EDRAK_VISUAL_VIEWER_HPP__

#include "Edrak/Images/Frame.hpp"
#include "Edrak/SLAM/Map.hpp"
#include <pangolin/pangolin.h>
#include <thread>

namespace Edrak {
class Viewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Viewer> Ptr;

  Viewer();

  void SetMap(Map::SharedPtr map) { map_ = map; }

  void Close();

  void AddCurrentFrame(StereoFrame::SharedPtr current_frame);

  void UpdateMap();

private:
  void ThreadLoop();

  void DrawFrame(StereoFrame::SharedPtr frame, const float *color);

  void DrawMapPoints();

  void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

  /// plot the features in current frame into an image
  cv::Mat PlotFrameImage();

  StereoFrame::SharedPtr current_frame_ = nullptr;
  Map::SharedPtr map_ = nullptr;

  std::thread viewer_thread_;
  bool viewer_running_ = true;

  std::unordered_map<unsigned long, StereoFrame::SharedPtr> active_keyframes_;
  std::unordered_map<unsigned long, Landmark::SharedPtr> active_landmarks_;
  bool map_updated_ = false;

  std::mutex viewer_data_mutex_;
};

} // namespace Edrak

#endif