#include "../catch.hpp"
#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/SLAM/Frontend.hpp"
#include <iostream>

TEST_CASE("Test StereoInit null Frame", "Frontend::StereoInit") {
  // Edrak::Frontend fe;
  // REQUIRE_FALSE(fe.AddFrame(nullptr));
}

TEST_CASE("Test StereoInit", "Frontend::StereoInit") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "KITTI/VO/14";
  const int N_FRAMES = 100;

  Edrak::Images::CameraMatD kittiPinholeCamera{9.812178e+02, 9.758994e+02,
                                               6.900000e+02, 2.471364e+02};

  Edrak::StereoCamera cam{kittiPinholeCamera, kittiPinholeCamera, 0.54};
  Edrak::Frontend fe;
  fe.SetCamera(cam);
  Edrak::IO::MonoReader leftReader{imgs_path + "/image_0/*.png",
                                   Edrak::IO::ImageType::GRAY, false};
  Edrak::IO::MonoReader rightReader{imgs_path + "/image_1/*.png",
                                    Edrak::IO::ImageType::GRAY, false};

  for (size_t i = 0; i < N_FRAMES; i++) {
    Edrak::StereoFrame::SharedPtr frame = Edrak::StereoFrame::CreateFrame();
    leftReader.NextFrame(frame->imgData);
    rightReader.NextFrame(frame->rightImgData);
    REQUIRE(frame->imgData.size() == frame->rightImgData.size());
    fe.AddFrame(frame);
  }
}