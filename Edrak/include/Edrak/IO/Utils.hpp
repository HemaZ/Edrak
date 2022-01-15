#ifndef __EDRAK_IO_UTILS_H__
#define ____EDRAK_IO_UTILS_H__
#include <glob.h>
#include <string>
#include <vector>

namespace Edrak {
namespace IO {
/**
 * @brief Get the files paths using the provided glob pattern
 *
 * @param glob_pattern
 * @return std::vector<std::string> files paths
 */
std::vector<std::string> GetFiles(const std::string &glob_pattern);
} // namespace IO
} // namespace Edrak

#endif // __UTILS_H__