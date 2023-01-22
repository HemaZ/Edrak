#ifndef __EDRAK_POSEESTMATION_2D_H__
#define __EDRAK_POSEESTMATION_2D_H__
#include "Edrak/Images/Images.hpp"
#include "Edrak/Types/Types.hpp"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
namespace Edrak {
namespace VO {
using namespace Edrak::Images;
class PoseEstmation {
private:
  CameraMatD k_;
  bool initialized_ = false;
  cv::Mat lastFrame_;
  Features::KeyPoints::KeyPoints lastFrameKps_;
  cv::Mat lastFrameDescriptors_;
  Edrak::SE3D pose_;

public:
  PoseEstmation(const CameraMatD k) : k_(k) {}
  int Process(const cv::Mat &img);
  Edrak::SE3D Pose() const { return pose_; }
  int nInliers;
};

} // namespace VO

} // namespace Edrak

#endif // __EDRAK_POSEESTMATION_2D_H__