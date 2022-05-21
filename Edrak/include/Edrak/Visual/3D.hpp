#ifndef __EDRAK_VIZ__3D_H__
#define __EDRAK_VIZ__3D_H__
#include "Edrak/Types/Types.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace Edrak {
namespace Visual {
/**
 * @brief Draw 3D trajectory.
 *
 */
void DrawTrajectory(const Edrak::TrajectoryD &);
/**
 * @brief Draw 2 Trajectories together for comparision.
 *
 * @param gt First trajectory (ground truth trajectory).
 * @param est Second trajectory (estimated trajectory)
 */
void DrawTrajectory(const Edrak::TrajectoryD &gt,
                    const Edrak::TrajectoryD &est);
void DrawPointCloud(const Edrak::PointCloudID &);
void DrawPointCloud(const Edrak::PointCloudRGB &);
} // namespace Visual

} // namespace Edrak

#endif // __EDRAK_VIZ__3D_H__