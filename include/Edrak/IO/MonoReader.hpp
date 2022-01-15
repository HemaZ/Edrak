#ifndef __EDRAK_IO_MONOREADER_H__
#define __EDRAK_IO_MONOREADER_H__
#include "Reader.hpp"
#include "Utils.hpp"
namespace Edrak {
namespace IO {

class MonoReader : public Reader {
private:
  /* data */
public:
  MonoReader(const std::string &folder_path, ImageType type = ImageType::RGB,
             bool loop = true);
  virtual ~MonoReader();
  bool NextFrame(cv::Mat &);
};

} // namespace IO

} // namespace Edrak
#endif // __EDRAK_IO_MONOREADER_H__