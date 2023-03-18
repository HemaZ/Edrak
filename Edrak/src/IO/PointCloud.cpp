#include "Edrak/IO/PointCloud.hpp"
#define TINYPLY_IMPLEMENTATION
#include "Edrak/IO/tinyply.h"
using namespace tinyply;

namespace Edrak {
bool WritePLYFromLandmarks(const Map::LandmarksData &landmarks,
                           const std::string filePath) {

  std::ofstream of(filePath.c_str());
  if (of.fail()) {
    return false;
  }
  of << "ply" << '\n'
     << "format ascii 1.0" << '\n'
     << "element vertex " << landmarks.size() << '\n'
     << "property float x" << '\n'
     << "property float y" << '\n'
     << "property float z" << '\n'
     << "property uchar red" << '\n'
     << "property uchar green" << '\n'
     << "property uchar blue" << '\n'
     << "end_header" << std::endl;
  // Export the structure (i.e. 3D Points) as white points.

  for (const auto &landmark : landmarks) {
    auto pos = landmark.second->Position();
    of << pos.x() << ' ';
    of << pos.y() << ' ';
    of << pos.z() << ' ';
    of << " 255 255 255\n";
  }
  of.close();
  return true;
}
} // namespace Edrak
