#include "Edrak/IO/PointCloud.hpp"
#define TINYPLY_IMPLEMENTATION
#include "Edrak/IO/tinyply.h"
using namespace tinyply;

namespace Edrak {
void WritePLYFromLandmarks(const Map::LandmarksData &landmarks,
                           const std::string filePath) {
  std::filebuf fb_binary;
  fb_binary.open(filePath + "-asci.ply", std::ios::out);
  std::ostream outstream_binary(&fb_binary);
  if (outstream_binary.fail())
    throw std::runtime_error("failed to open " + filePath);

  tinyply::PlyFile plyOutputFile;
  std::vector<double> pts;
  pts.reserve(landmarks.size() * 3);
  for (const auto &landmark : landmarks) {
    auto pos = landmark.second->Position();
    pts.push_back(pos[0]);
    pts.push_back(pos[1]);
    pts.push_back(pos[2]);
  }
  plyOutputFile.add_properties_to_element(
      "vertex", {"x", "y", "z"}, Type::FLOAT64, landmarks.size(),
      reinterpret_cast<uint8_t *>(pts.data()), Type::INVALID, 0);
  // Write a binary file
  plyOutputFile.write(outstream_binary, false);
}
} // namespace Edrak
