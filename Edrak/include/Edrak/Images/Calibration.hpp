#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__
#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>
namespace Edrak {
namespace Images {
/**
 * @brief Structure represinting Camera intrinsics matrix.
 *  Coefficients order, fx,fy,cx,cy.
 * @tparam T Type float or double.
 */
template <typename T> struct CameraMatrix {
public:
  T fx;
  T fy;
  T cx;
  T cy;
  T x(int u, T d = 1.0) const { return d * (u - cx) / fx; }
  T y(int v, T d = 1.0) const { return d * (v - cy) / fy; }
  T u(T x) const { return fx * x + cx; }
  T v(T y) const { return fy * y + cy; }
  static CameraMatrix<T> FromMat(const Eigen::Matrix<T, 3, 3> &mat) {
    return CameraMatrix<T>{mat(0, 0), mat(1, 1), mat(0, 2), mat(1, 2)};
  }
};
using CameraMatF = CameraMatrix<float>;
using CameraMatD = CameraMatrix<double>;

/**
 * @brief Struct holds coefficients for rad-tan distortion model in this order
 * k1,k2,p1,p2.
 * @tparam T Type float or double.
 */
template <typename T> struct RadTanCoeffs {
public:
  T k1;
  T k2;
  T p1;
  T p2;
};
using RadTanCoeffsF = RadTanCoeffs<float>;
using RadTanCoeffsD = RadTanCoeffs<double>;

cv::Mat Undistort(const cv::Mat &src, const CameraMatD &A,
                  const RadTanCoeffsD &rad_tan);
} // namespace Images

} // namespace Edrak

#endif // __CALIBRATION_H__