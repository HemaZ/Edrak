#include "Edrak/IO/Utils.hpp"
namespace Edrak {
namespace IO {
std::vector<std::string> GetFiles(const std::string &glob_pattern) {
  std::vector<std::string> files;
  glob_t glob_result;
  int ret =
      glob(glob_pattern.c_str(), GLOB_TILDE | GLOB_MARK, NULL, &glob_result);
  if (ret == 0) {
    for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
      files.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
  } else {
  }
  return files;
}
} // namespace IO

} // namespace Edrak