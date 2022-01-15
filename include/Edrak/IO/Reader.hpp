#ifndef __EDRAK_IO_READER_H__
#define __EDRAK_IO_READER_H__
#include <opencv2/imgcodecs.hpp>
#include <string>
namespace Edrak {
namespace IO {
enum class ImageType { RGB, GRAY };
class Reader {
protected:
  std::string folderPath;
  std::vector<std::string> framesPaths;
  ImageType imageType;
  bool end = true;
  bool loop = true;
  std::size_t currentFrameIdx = -1;

public:
  Reader(const std::string &folder_path, ImageType type = ImageType::RGB,
         bool loop = true)
      : folderPath(folder_path), imageType(type), loop{loop} {}
  virtual ~Reader() {}
  virtual bool NextFrame(cv::Mat &) = 0;
};

} // namespace IO

} // namespace Edrak

#endif // __EDRAK_IO_READER_H__