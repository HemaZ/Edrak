#ifndef __EDRAK_IMAGE_RGBD_H__
#define __EDRAK_IMAGE_RGBD_H__
#include "Edrak/Images/Calibration.hpp"
#include "Edrak/Types/Types.hpp"
#include <opencv2/core.hpp>
namespace Edrak {
namespace Images {
/**
 * @brief
 *
 * @param rgb
 * @param depth
 * @param intrinsics
 * @return Edrak::PointCloudRGB
 */
Edrak::PointCloudRGB
RGBDToPointCloud(const cv::Mat &rgb, const cv::Mat &depth,
                 const Edrak::Images::CameraMatD &intrinsics,
                 double depthScale = 1000,
                 const Edrak::SE3D = Edrak::SE3D());
} // namespace Images

} // namespace Edrak

#endif // __EDRAK_IMAGE_RGBD_H__