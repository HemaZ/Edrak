#ifndef __EDRAK_VIZ__3D_H__
#define __EDRAK_VIZ__3D_H__
#include "Edrak/Types/Transformation.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace Edrak {
namespace Visual {
/**
 * @brief Draw 3D trajectory.
 *
 */
void DrawTrajectory(const Edrak::Types::TrajectoryD &);
/**
 * @brief Draw 2 Trajectories together for comparision.
 *
 * @param gt First trajectory (ground truth trajectory).
 * @param est Second trajectory (estimated trajectory)
 */
void DrawTrajectory(const Edrak::Types::TrajectoryD &gt,
                    const Edrak::Types::TrajectoryD &est);
void DrawPointCloud(const Edrak::Types::PointCloudID &);
void DrawPointCloud(const Edrak::Types::PointCloudRGB &);
} // namespace Visual

} // namespace Edrak

#endif // __EDRAK_VIZ__3D_H__