#ifndef __EDRAK_IO_TRAJECTORY_H__
#define __EDRAK_IO_TRAJECTORY_H__
#include "Edrak/Exceptions/Exceptions.hpp"
#include "Edrak/Types/Types.hpp"
#include <fstream>
#include <sophus/se3.hpp>
namespace Edrak {
namespace IO {
/**
 * @brief Load trajectory from CSV file. each row should has timestamp, x, y, z,
 *  quaterion_x,quaterion_y,quaterion_z, quaterion_w
 *
 * @param path CSV file path.
 * @return Edrak::TrajectoryD Trajectory as SE3D poses.
 */
Edrak::TrajectoryD LoadTrajectory(const std::string &path);

/**
 * @brief Export trajectory as TUM file format.
 *
 * @param path TUM file path to write the trajectory to.
 * @param trajectory Trajectory to Save.
 * @param writeHeader Save header to the trajectory file.
 * @return bool If success.
 */
bool ExportTrajectory(const std::string &path,
                      const Edrak::TrajectoryD &trajectory,
                      bool writeHeader = false);
} // namespace IO

} // namespace Edrak

#endif // __EDRAK_IO_TRAJECTORY_H__