#include "Edrak/Images/Calibration.hpp"
#include <fstream>

namespace Edrak {
namespace Images {
cv::Mat Undistort(const cv::Mat &src, const CameraMatD &A,
                  const RadTanCoeffsD &rad_tan) {
  cv::Mat res(src.size(), src.type());
  int rows = src.rows;
  int cols = src.cols;
  int c = src.channels();
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      double x = A.x(u);
      double y = A.y(v);
      double r = sqrt(x * x + y * y);
      double r2 = r * r;
      double r4 = r2 * r * r;
      double x_distorted = x * (1 + rad_tan.k1 * r2 + rad_tan.k2 * r4) +
                           2 * rad_tan.p1 * x * y +
                           rad_tan.p2 * (r2 + 2 * x * x);

      double y_distorted = y * (1 + rad_tan.k1 * r2 + rad_tan.k2 * r4) +
                           rad_tan.p1 * (r2 + 2 * y * y) +
                           2 * rad_tan.p2 * x * y;

      double u_distorted = A.u(x_distorted);
      double v_distorted = A.v(y_distorted);
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols &&
          v_distorted < rows) {
        if (c == 1) {
          res.at<uchar>(v, u) =
              src.at<uchar>((int)v_distorted, (int)u_distorted);
        } else if (c == 3) {
          res.at<cv::Vec3b>(v, u) =
              src.at<cv::Vec3b>((int)v_distorted, (int)u_distorted);
        }
      } else {
        res.at<uchar>(v, u) = 0;
      }
    }
  }
  return res;
}
} // namespace Images

std::vector<CameraModel> ParseKITTICameras(
    const std::string &calibPath) { // read camera intrinsics and extrinsics
  std::ifstream fin(calibPath);
  if (!fin) {
    throw std::runtime_error("Cannot open calibration file in path " +
                             calibPath);
  }
  std::vector<CameraModel> cameras;
  for (int i = 0; i < 4; ++i) {
    char camera_name[3];
    for (int k = 0; k < 3; ++k) {
      fin >> camera_name[k];
    }
    double projection_data[12];
    for (int k = 0; k < 12; ++k) {
      fin >> projection_data[k];
    }
    Eigen::Matrix3d K;
    K << projection_data[0], projection_data[1], projection_data[2],
        projection_data[4], projection_data[5], projection_data[6],
        projection_data[8], projection_data[9], projection_data[10];
    Eigen::Vector3d t;
    t << projection_data[3], projection_data[7], projection_data[11];
    t = K.inverse() * t;
    K = K * 0.5;
    CameraModel camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                       Sophus::SE3d(Sophus::SO3d(), t));

    cameras.push_back(camera);
  }
  fin.close();
  return cameras;
}
} // namespace Edrak