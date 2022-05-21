#ifndef __EDRAK_IO_TRAJECTORY_H__
#define __EDRAK_IO_TRAJECTORY_H__
#include "Edrak/Exceptions/Exceptions.hpp"
#include "Edrak/Types/Types.hpp"
#include <fstream>
#include <sophus/se3.hpp>
namespace Edrak {
namespace IO {
Edrak::TrajectoryD LoadTrajectory(const std::string path);
} // namespace IO

} // namespace Edrak

#endif // __EDRAK_IO_TRAJECTORY_H__