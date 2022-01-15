#include "Edrak/IO/MonoReader.hpp"
namespace Edrak {

namespace IO {

MonoReader::MonoReader(const std::string &folder_path, ImageType type,
                       bool loop)
    : Reader{folder_path, type, loop} {
  framesPaths = GetFiles(folderPath);
  currentFrameIdx = 0;
  end = false;
}

MonoReader::~MonoReader() {}

bool MonoReader::NextFrame(cv::Mat &img) {
  if (currentFrameIdx == framesPaths.size()) {
    end = true;
  }
  if (end) {
    if (loop) {
      currentFrameIdx = 0;
      end = false;
    } else {
      return false;
    }
  }
  if (imageType == ImageType::RGB)
    img = cv::imread(framesPaths[currentFrameIdx++], cv::IMREAD_COLOR);
  else {
    img = cv::imread(framesPaths[currentFrameIdx++], cv::IMREAD_GRAYSCALE);
  }
  return true;
}

} // namespace IO

} // namespace Edrak
