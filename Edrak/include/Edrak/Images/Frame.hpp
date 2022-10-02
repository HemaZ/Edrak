#ifndef __EDRAK_IMAGES_FRAME_H__
#define __EDRAK_IMAGES_FRAME_H__
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <sophus/se3.hpp>
namespace Edrak {
namespace Images {
namespace Features {
struct Feature;
}
} // namespace Images
struct Frame {
private:
  std::mutex poseMutex;
  std::mutex dataMutex;
  // Tcw
  Sophus::SE3d pose;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Frame> SharedPtr;
  bool isKeyFrame;
  uint32_t frameId;
  uint32_t keyFrameId;
  uint64_t timeStamp;
  // Frame's data. in case of stereo frame, this represents the left image.
  cv::Mat imgData;
  // Image's features
  std::vector<std::shared_ptr<Edrak::Images::Features::Feature>> features;
  // Image's ORB descriptors
  cv::Mat orbDescriptors;
  Frame() {}
  /**
   * @brief Construct a new Frame object
   *
   * @param id
   * @param timestamp
   * @param pose
   * @param img
   */
  Frame(uint32_t id, uint64_t timestamp, const Sophus::SE3d &pose,
        const cv::Mat &img);
  /**
   * @brief Tcw
   *
   * @return Sophus::SE3d
   */
  Sophus::SE3d Pose() {
    std::lock_guard<std::mutex> lock(poseMutex);
    return pose;
  }
  /**
   * @brief
   *
   * @param newPose
   */
  void Pose(const Sophus::SE3d &newPose) {
    std::lock_guard<std::mutex> lock(poseMutex);
    pose = newPose;
  }

  /**
   * @brief Create a Frame object
   *
   * @return std::shared_ptr<Frame>
   */
  static std::shared_ptr<Frame> CreateFrame();
};

struct StereoFrame : public Frame {
  using SharedPtr = std::shared_ptr<StereoFrame>;
  cv::Mat rightImgData;
  // Right image's features
  std::vector<std::shared_ptr<Edrak::Images::Features::Feature>> featuresRight;
  // Right images orb descriptors
  cv::Mat rightImgDescriptors;
  static std::shared_ptr<StereoFrame> CreateFrame();
};
} // namespace Edrak

#endif // __EDRAK_IMAGES_FRAME_H__