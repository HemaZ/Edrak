#ifndef __EDRAK_IO_POINTCLOUD_H__
#define __EDRAK_IO_POINTCLOUD_H__
#include "Edrak/SLAM/Map.hpp"
#include <fstream>
namespace Edrak {
void WritePLYFromLandmarks(const Map::LandmarksData &landmarks,
                           const std::string filePath);

}
#endif // __EDRAK_IO_POINTCLOUD_H__