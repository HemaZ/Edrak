#include "Edrak/IO/Trajectory.hpp"
namespace Edrak {

namespace IO {
Edrak::TrajectoryD LoadTrajectory(const std::string &path) {
  std::ifstream fin(path);
  Edrak::TrajectoryD trajectory;
  if (!fin) {
    throw Edrak::Exceptions::FileNotFound("[Edrak] " + path + " Not found!");
  }
  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    trajectory.emplace_back(
        Edrak::SE3D(Edrak::QuatD(qw, qx, qy, qz), Edrak::Vector3d(tx, ty, tz)));
  }
  return trajectory;
}

bool ExportTrajectory(const std::string &path,
                      const Edrak::TrajectoryD &trajectory, bool writeHeader) {
  std::ofstream of(path.c_str());
  if (of.fail()) {
    return false;
  }
  if (writeHeader) {
    of << "t x y z quat_x quat_y quat_z quat_w\n";
  }
  for (const auto &pose : trajectory) {
    Sophus::Vector3d trans = pose.translation();
    Eigen::Vector4d quat = pose.so3().params();
    of << 0 << " ";
    of << trans.x() << " " << trans.y() << " " << trans.z() << " ";
    of << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
       << "\n";
  }
  of.close();
  return true;
}
} // namespace IO

} // namespace Edrak
