#include "Edrak/Images/Calibration.hpp"
#include "Eigen/Core"
#include "ceres/autodiff_cost_function.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#ifndef __EDRAK__FRONTEND__CERES__PROBLEMS
#define __EDRAK__FRONTEND__CERES__PROBLEMS
using namespace Edrak::Images;

struct EigenReprojectionError {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EigenReprojectionError(double x_observed, double y_observed,
                         Eigen::Vector3d &pt3d)
      : x_(x_observed), y_(y_observed), p_vec3(pt3d) {}

  template <typename T>
  bool operator()(const T *const trans_ptr, const T *const quat_ptr,
                  T *residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(trans_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_a(quat_ptr);
    Sophus::SE3d T_pose(q_a, p_a);
    Eigen::Matrix<T, 3, 1> pos_pixel = k_ * (T_pose * p_vec3);
    pos_pixel /= pos_pixel[2];
    // The error is the difference between the predicted and observed position.
    residuals[0] = pos_pixel[0] - x_;
    residuals[1] = pos_pixel[1] - y_;
    return true;
  }

private:
  double x_;
  double y_;
  Eigen::Matrix<double, 3, 3> k_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_vec3;
};

struct ReprojectionError {
  ReprojectionError(double x_observed, double y_observed,
                    CameraMatD calibration, const double *point3dArr)
      : xObserved(x_observed), yObserved(y_observed), calibration(calibration) {
    point3d[0] = point3dArr[0];
    point3d[1] = point3dArr[1];
    point3d[2] = point3dArr[2];
  }
  template <typename T>
  bool operator()(const T *const quat, const T *const tran,
                  T *residuals) const {
    T p[3];
    T point[] = {T(point3d[0]), T(point3d[1]), T(point3d[2])};
    // Rotate the point using the angle axis pose[0],pose[1],pose[2]
    ceres::QuaternionRotatePoint(quat, point, p);
    // Apply translation to the point
    p[0] += tran[0];
    p[1] += tran[1];
    p[2] += tran[2];

    p[0] /= p[2];
    p[1] /= p[2];

    T predicted_x = T(calibration.fx) * p[0] + T(calibration.cx);
    T predicted_y = T(calibration.fy) * p[1] + T(calibration.cy);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(xObserved);
    residuals[1] = predicted_y - T(yObserved);
    return true;
  }

  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y,
                                     const CameraMatD calibration,
                                     const double *point3dArr) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>(
        new ReprojectionError(observed_x, observed_y, calibration,
                              point3dArr)));
  }

private:
  double xObserved;
  double yObserved;
  CameraMatD calibration;
  double point3d[3];
};

struct ReprojectionErrorBA {
  ReprojectionErrorBA(double x_observed, double y_observed,
                      CameraMatD calibration)
      : xObserved(x_observed), yObserved(y_observed), calibration(calibration) {
  }
  template <typename T>
  bool operator()(const T *const quat, const T *const tran,
                  const T *const point3d, T *residuals) const {
    T p[3];
    // Rotate the point using the angle axis pose[0],pose[1],pose[2]
    ceres::QuaternionRotatePoint(quat, point3d, p);
    // Apply translation to the point
    p[0] += tran[0];
    p[1] += tran[1];
    p[2] += tran[2];

    p[0] /= p[2];
    p[1] /= p[2];

    T predicted_x = T(calibration.fx) * p[0] + T(calibration.cx);
    T predicted_y = T(calibration.fy) * p[1] + T(calibration.cy);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(xObserved);
    residuals[1] = predicted_y - T(yObserved);
    return true;
  }

  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y,
                                     const CameraMatD calibration) {
    return (new ceres::AutoDiffCostFunction<ReprojectionErrorBA, 2, 4, 3, 3>(
        new ReprojectionErrorBA(observed_x, observed_y, calibration)));
  }

private:
  double xObserved;
  double yObserved;
  CameraMatD calibration;
};

#endif