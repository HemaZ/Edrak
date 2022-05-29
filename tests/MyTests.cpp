
// This tells Catch to provide a main() - only do this in one cpp file
#define CATCH_CONFIG_MAIN
#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/IO/Utils.hpp"
#include <opencv2/opencv.hpp>

#include "catch.hpp"
#include <iostream>
#include <limits>
#include <type_traits>

// This tests the output of the `Edrak::IO::GetFiles` function
TEST_CASE("correct files are returned", "Edrak::IO::GetFiles") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string img_path = data_dir + "VO";
  std::string glob_pattern{img_path + "/*.png"};
  std::vector<std::string> paths = Edrak::IO::GetFiles(glob_pattern);
  CHECK(paths.size() == 11);
}

// This tests the correct out_of_range exceptions are generated
TEST_CASE("NextFrames returning correct number of frames",
          "Edrak::IO::MonoReader::nextFrame") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string img_path = data_dir + "VO";
  Edrak::IO::MonoReader reader{img_path + "/*.png", Edrak::IO::ImageType::GRAY,
                               false};
  cv::Mat frame;
  while (reader.NextFrame(frame)) {
    REQUIRE_FALSE(!frame.data);
    // cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display Image", frame);
    // cv::waitKey(0);
  }
}
