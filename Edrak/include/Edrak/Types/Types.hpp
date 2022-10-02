#ifndef __EDRAK__Transformation_H__
#define __EDRAK__Transformation_H__
#include "Edrak/Exceptions/Exceptions.hpp"
#include "Eigen/Core"
#include "Eigen/Geometry"
#define SOPHUS_USE_BASIC_LOGGING
#include "sophus/se3.hpp"
#include <vector>
namespace Edrak {
using QuatD = Eigen::Quaternion<double>;
using QuatF = Eigen::Quaternion<float>;
/**
 * @brief Transformation matrix, This type represents double SO(3)
 *
 */
using TransMatD = Eigen::Matrix3d;
/**
 * @brief Transformation matrix, This type represents float SO(3)
 *
 */
using TransMatF = Eigen::Matrix3f;
using SE3D = Sophus::SE3d;
using SE3F = Sophus::SE3f;
using SO3D = Sophus::SO3d;
using TrajectoryD = std::vector<SE3D, Eigen::aligned_allocator<SE3D>>;
using TrajectoryF = std::vector<SE3F, Eigen::aligned_allocator<SE3F>>;
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Point3D = Eigen::Vector3d;
using Point3F = Eigen::Vector3f;
using Points3D = Eigen::MatrixX3d;

using PointCloudID =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;
using PointCloudRGB = std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>>;
/**
 * @brief
 *
 * @tparam T
 * @param r
 * @param p
 * @param y
 * @param res
 */
template <typename T> void RPYToQuat(T r, T p, T y, Eigen::Quaternion<T> &res) {
  using vector = Eigen::Matrix<T, 3, 1>;
  res = Eigen::AngleAxis<T>(r, vector::UnitX()) *
        Eigen::AngleAxis<T>(p, vector::UnitY()) *
        Eigen::AngleAxis<T>(y, vector::UnitZ());
  res.normalize();
}

} // namespace Edrak

#endif //__EDRAK__Transformation_H__