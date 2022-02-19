#include "Edrak/Images/Calibration.hpp"
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
} // namespace Edrak