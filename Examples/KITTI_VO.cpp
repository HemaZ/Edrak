#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/SLAM/Frontend.hpp"
#include "Edrak/SLAM/Viewer.hpp"
#include <memory>
int main(int argc, char const *argv[]) {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "KITTI/VO/14";
  const int N_FRAMES = 100;

  Edrak::Images::CameraMatD kittiPinholeCamera{9.812178e+02, 9.758994e+02,
                                               6.900000e+02, 2.471364e+02};

  Edrak::StereoCamera cam{kittiPinholeCamera, kittiPinholeCamera, 0.54};
  Edrak::Frontend fe;
  Edrak::Map::SharedPtr map = std::make_shared<Edrak::Map>();
  fe.SetMap(map);

  // Create Viewer
  // Edrak::Viewer::SharedPtr viewer = std::make_shared<Edrak::Viewer>();
  // viewer->SetMap(map);
  // fe.SetViewer(viewer);

  fe.SetCamera(cam);
  Edrak::IO::MonoReader leftReader{imgs_path + "/image_0/*.png",
                                   Edrak::IO::ImageType::GRAY, false};
  Edrak::IO::MonoReader rightReader{imgs_path + "/image_1/*.png",
                                    Edrak::IO::ImageType::GRAY, false};

  for (size_t i = 0; i < N_FRAMES; i++) {
    Edrak::StereoFrame::SharedPtr frame = Edrak::StereoFrame::CreateFrame();
    leftReader.NextFrame(frame->imgData);
    rightReader.NextFrame(frame->rightImgData);
    if (frame->imgData.size() == frame->rightImgData.size()) {
      fe.AddFrame(frame);
    } else {
      break;
    }
  }
  return 0;
}
