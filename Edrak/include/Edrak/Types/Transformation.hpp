#ifndef __EDRAK__Transformation_H__
#define __EDRAK__Transformation_H__
#include "Edrak/Exceptions/Exceptions.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <vector>
namespace Edrak {
namespace Types {
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
using SE3D = Eigen::Isometry3d;
using SE3F = Eigen::Isometry3f;
using TrajectoryD =
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;
using TrajectoryF =
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>;
template <typename T> void RPYToQuat(T r, T p, T y, Eigen::Quaternion<T> &res) {
  using vector = Eigen::Matrix<T, 3, 1>;
  res = Eigen::AngleAxis<T>(r, vector::UnitX()) *
        Eigen::AngleAxis<T>(p, vector::UnitY()) *
        Eigen::AngleAxis<T>(y, vector::UnitZ());
  res.normalize();
}
} // namespace Types

} // namespace Edrak

#endif //__EDRAK__Transformation_H__