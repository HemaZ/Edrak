#include "Edrak/Images/Features.hpp"
#include "Edrak/Exceptions/Exceptions.hpp"

namespace Edrak {
namespace Images {
namespace Features {
void ORB(const cv::Mat &img, const KeyPoints::KeyPoints &kps,
         Descriptors::Descriptors<Descriptors::BRIEF> &descriptors) {
  throw Edrak::Exceptions::UnsupportedModeException();
}
} // namespace Features

} // namespace Images
} // namespace Edrak