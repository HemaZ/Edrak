#ifndef __EDRAK_IMAGES_TRIANGULATION_HPP__
#define __EDRAK_IMAGES_TRIANGULATION_HPP__
#include "sophus/se3.hpp"
#include <eigen3/Eigen/Core>
#include <vector>
namespace Edrak {
namespace Images {
inline bool Triangulation(const std::vector<Sophus::SE3d> &poses,
                          const std::vector<cv::Point2f> &points,
                          Eigen::Vector3d &resPoint) {
  Eigen::MatrixXd A(2 * poses.size(), 4);
  Eigen::VectorXd b(2 * poses.size());
  b.setZero();
  for (size_t i = 0; i < poses.size(); ++i) {
    Eigen::Matrix<double, 3, 4> m = poses[i].matrix3x4();
    A.block<1, 4>(2 * i, 0) = points[i].x * m.row(2) - m.row(0);
    A.block<1, 4>(2 * i + 1, 0) = points[i].y * m.row(2) - m.row(1);
  }
  auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  resPoint = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

  if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {

    return true;
  }
  return false;
}
} // namespace Images
} // namespace Edrak

#endif // __EDRAK_IMAGES_TRIANGULATION_HPP__
