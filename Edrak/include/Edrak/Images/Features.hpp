#ifndef __EDRAK_IMAGE_FEATURES_H__
#define __EDRAK_IMAGE_FEATURES_H__

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d/features2d.hpp>
namespace Edrak {
struct StereoFrame;
struct Landmark;
namespace Images {
namespace Features {
struct Feature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using SharedPtr = std::shared_ptr<Feature>;
  // Frame which contains this feature.
  std::weak_ptr<Edrak::StereoFrame> frame;
  // 3D Point location of this feature.
  std::weak_ptr<Edrak::Landmark> landmark;
  // 2D location of the feature on the frame.
  cv::KeyPoint position;
  bool isOutlier = false;
  bool matchedOnLeftImg = false;
  /**
   * @brief Construct a new Feature object
   *
   */
  Feature() {}

  /**
   * @brief Construct a new Feature object
   *
   * @param frame Associated frame.
   * @param kp Feature location.
   */
  Feature(std::shared_ptr<StereoFrame> frame, const cv::KeyPoint &kp)
      : frame(frame), position(kp) {}
};
namespace Descriptors {
/**
 * @brief Datatype to represent the BRIEF descriptor.
 *
 */
using BRIEF = uint32_t[8];

template <typename T> using Descriptors = std::vector<T>;
} // namespace Descriptors

namespace KeyPoints {
/**
 * @brief Datatype to represent a 2D keypoint.
 *
 */
using KeyPoint = cv::KeyPoint;
/**
 * @brief Datatype to represent a vector of 2D keypoint.
 *
 */
using KeyPoints = std::vector<KeyPoint>;

/**
 * @brief Compute the FAST keypoints.
 *
 * @param img Input gray image.
 * @param kps Vector to hold the output keypoints.
 * @param threshold Threshold on difference between intensity of the central
 * pixel and pixels of a circle around this pixel.
 * @param non_max_suppresion If true, non-maximum suppression is applied to
 * detected corners (keypoints).
 */
void FAST(const cv::Mat &img, KeyPoints &kps, int threshold,
          bool non_max_suppresion);

} // namespace KeyPoints

using cv::DMatch;
/**
 * @brief Vector of 2D Matches. @sa cv::DMatch
 *
 */
using Matches2D = std::vector<DMatch>;

/**
 * @brief Computes ORB descriptors and FAST angles.
 *
 * @param img Input Image.
 * @param kps KeyPoints.
 * @param descriptors ORB descriptors.
 */
void ORB(const cv::Mat &img, KeyPoints::KeyPoints &kps, cv::Mat &descriptors);
/**
 * @brief
 *
 * @param img1
 * @param img2
 * @param kps1
 * @param kps2
 * @param matches
 */
void ExtractORBMatches(const cv::Mat &img1, const cv::Mat &img2,
                       KeyPoints::KeyPoints &kps1, KeyPoints::KeyPoints &kps2,
                       Matches2D &matches);
/**
 * @brief Keypoints matching using descriptors and BruteForce-Hamming Algorithm.
 *
 * @param descriptors1 First keypoints' descriptors.
 * @param descriptors2 Second keypoints' descriptors.
 * @param matches Matched keypoints.
 */
void FeaturesMatching(const cv::Mat &descriptors1, const cv::Mat &descriptors2,
                      Matches2D &matches);
/**
 * @brief Filter Matches which have distance < (max->distance - min->distance) *
 * ratio.
 *
 * @param matches [Input] 2D Keypoints matches.
 * @param filteredMatches [Output] Filtered matches.
 * @param ratio filtering [Input] ratio.
 */
void FilterMatches(const Matches2D &matches, Matches2D &filteredMatches,
                   float ratio = 0.3);

/**
 * @brief Extract Points2f from Matches.
 *
 * @param matches
 * @param pts1
 * @param pts2
 */
inline void MatchesToPoints(const Matches2D &matches,
                            const KeyPoints::KeyPoints &keyPoints1,
                            const KeyPoints::KeyPoints &keyPoints2,
                            std::vector<cv::Point2f> &pts1,
                            std::vector<cv::Point2f> &pts2) {
  for (const auto &match : matches) {
    pts1.push_back(keyPoints1[match.queryIdx].pt);
    pts2.push_back(keyPoints2[match.trainIdx].pt);
  }
}
} // namespace Features

} // namespace Images

} // namespace Edrak

#endif // __EDRAK_IMAGE_FEATURES_H__