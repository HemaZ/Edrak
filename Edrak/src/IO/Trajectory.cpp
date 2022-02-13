#include "Edrak/IO/Trajectory.hpp"
namespace Edrak {

namespace IO {
Edrak::Types::TrajectoryD LoadTrajectory(const std::string path) {
  std::ifstream fin(path);
  Edrak::Types::TrajectoryD trajectory;
  if (!fin) {
    throw Edrak::Exceptions::FileNotFound("[Edrak] " + path + " Not found!");
  }
  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    trajectory.emplace_back(
        Edrak::Types::SE3D(Edrak::Types::QuatD(qw, qx, qy, qz),
                           Edrak::Types::Vector3d(tx, ty, tz)));
  }
  return trajectory;
}
} // namespace IO

} // namespace Edrak
