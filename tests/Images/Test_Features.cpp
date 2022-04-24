#include "../catch.hpp"
#include "Edrak/Benchamrk/Timer.hpp"
#include "Edrak/Images/Features.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

TEST_CASE("FAST_TEST", "KeyPoints") {
  using namespace Edrak::Images::Features;
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  cv::Mat im1 = cv::imread(data_dir + "Images/Orb/0000000026.png");
  KeyPoints::KeyPoints kps;
  KeyPoints::FAST(im1, kps, 40, true);
}