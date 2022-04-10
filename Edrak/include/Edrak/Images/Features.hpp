#include <cstdint>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace Edrak {
namespace Images {
namespace Features {

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
          bool non_max_suppresion) {
  cv::FAST(img, kps, threshold, non_max_suppresion);
}

} // namespace KeyPoints

/**
 * @brief Computes ORB descriptors and FAST angles.
 *
 * @param img Input Image.
 * @param kps KeyPoints.
 * @param descriptors ORB descriptors.
 */
void ORB(const cv::Mat &img, const KeyPoints::KeyPoints &kps,
         Descriptors::Descriptors<Descriptors::BRIEF> &descriptors);

} // namespace Features

} // namespace Images

} // namespace Edrak
