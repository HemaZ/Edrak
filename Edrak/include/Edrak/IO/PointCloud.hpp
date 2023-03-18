#ifndef __EDRAK_IO_POINTCLOUD_H__
#define __EDRAK_IO_POINTCLOUD_H__
#include "Edrak/SLAM/Map.hpp"
#include <fstream>
namespace Edrak {
/**
 * @brief Export Map landmarks as PLY file.
 *
 * @param landmarks
 * @param filePath
 * @return true
 * @return false
 */
bool WritePLYFromLandmarks(const Map::LandmarksData &landmarks,
                           const std::string filePath);

} // namespace Edrak
#endif // __EDRAK_IO_POINTCLOUD_H__