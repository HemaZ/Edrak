
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
TEST_CASE("correct files are returned", "[Edrak::IO::GetFiles]") {
  std::string glob_pattern{
      "../tests/2011_09_26/2011_09_26_drive_0002_sync/image_00/data/*.png"};
  std::vector<std::string> paths = Edrak::IO::GetFiles(glob_pattern);
  CHECK(paths[0] == "../tests/2011_09_26/2011_09_26_drive_0002_sync/image_00/"
                    "data/0000000000.png");
  CHECK(paths[1] == "../tests/2011_09_26/2011_09_26_drive_0002_sync/image_00/"
                    "data/0000000001.png");
  CHECK(paths[2] == "../tests/2011_09_26/2011_09_26_drive_0002_sync/image_00/"
                    "data/0000000002.png");
  CHECK(paths[76] == "../tests/2011_09_26/2011_09_26_drive_0002_sync/image_00/"
                     "data/0000000076.png");
}

// This tests the correct out_of_range exceptions are generated
TEST_CASE("NextFrames returning correct number of frames",
          "[Edrak::IO::MonoReader::nextFrame]") {
  Edrak::IO::MonoReader reader{
      "../tests/2011_09_26/2011_09_26_drive_0002_sync/image_00/data/*.png",
      Edrak::IO::ImageType::GRAY, false};
  cv::Mat frame;
  while (reader.NextFrame(frame)) {
    REQUIRE_FALSE(!frame.data);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", frame);
    cv::waitKey(0);
  }
}

TEST_CASE("NextFrames returning correct number of frames(RGB)",
          "[Edrak::IO::MonoReader::nextFrame]") {
  Edrak::IO::MonoReader reader{
      "../tests/2011_09_26/2011_09_26_drive_0002_sync/image_02/data/*.png",
      Edrak::IO::ImageType::RGB, false};
  cv::Mat frame;
  while (reader.NextFrame(frame)) {
    REQUIRE_FALSE(!frame.data);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", frame);
    cv::waitKey(0);
  }
}