#include "Edrak/Images/RGBD.hpp"
namespace Edrak {
namespace Images {
Edrak::Types::PointCloudRGB
RGBDToPointCloud(const cv::Mat &rgb, const cv::Mat &depth,
                 const Edrak::Images::CameraMatD &intrinsics, double depthScale,
                 const Edrak::Types::SE3D Twc) {
  Edrak::Types::PointCloudRGB pointcloud;
  pointcloud.reserve(rgb.cols * rgb.rows);
  for (int v = 0; v < rgb.rows; v++) {
    for (int u = 0; u < rgb.cols; u++) {
      unsigned int d = depth.ptr<unsigned short>(v)[u];
      if (d == 0)
        continue;
      Edrak::Types::Vector3d point;
      point[2] = double(d) / depthScale;     // z
      point[0] = point[2] * intrinsics.x(u); // x
      point[1] = point[2] * intrinsics.y(v); // y
      point = Twc * point;                   // Transformed to world frame

      Edrak::Types::Vector6d p;
      p.head<3>() = point;
      p[5] = rgb.data[v * rgb.step + u * rgb.channels()];     // blue
      p[4] = rgb.data[v * rgb.step + u * rgb.channels() + 1]; // green
      p[3] = rgb.data[v * rgb.step + u * rgb.channels() + 2]; // red
      pointcloud.push_back(p);
    }
  }
  return pointcloud;
}
} // namespace Images

} // namespace Edrak
