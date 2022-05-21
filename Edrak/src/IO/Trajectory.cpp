#include "Edrak/IO/Trajectory.hpp"
namespace Edrak {

namespace IO {
Edrak::TrajectoryD LoadTrajectory(const std::string path) {
  std::ifstream fin(path);
  Edrak::TrajectoryD trajectory;
  if (!fin) {
    throw Edrak::Exceptions::FileNotFound("[Edrak] " + path + " Not found!");
  }
  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    trajectory.emplace_back(
        Edrak::SE3D(Edrak::QuatD(qw, qx, qy, qz),
                           Edrak::Vector3d(tx, ty, tz)));
  }
  return trajectory;
}
} // namespace IO

} // namespace Edrak
