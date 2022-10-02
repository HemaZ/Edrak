#include "Edrak/Images/Frame.hpp"

namespace Edrak {
Frame::Frame(uint32_t id, uint64_t timestamp, const Sophus::SE3d &pose,
             const cv::Mat &img)
    : frameId(id), timeStamp(timestamp), imgData(img), pose(pose) {}

std::shared_ptr<Frame> Frame::CreateFrame() {
  static uint32_t newFrameId = 0;
  std::shared_ptr<Frame> ptr = std::make_shared<Frame>();
  ptr->frameId = newFrameId;
  ++newFrameId;
  return ptr;
}

std::shared_ptr<StereoFrame> StereoFrame::CreateFrame() {
  static uint32_t newFrameId = 0;
  std::shared_ptr<StereoFrame> ptr = std::make_shared<StereoFrame>();
  ptr->frameId = newFrameId;
  ++newFrameId;
  return ptr;
}

} // namespace Edrak