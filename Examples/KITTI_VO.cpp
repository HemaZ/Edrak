#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/SLAM/Frontend.hpp"
#include "Edrak/SLAM/Viewer.hpp"
#include <memory>
int main(int argc, char const *argv[]) {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "KITTI/VO/16";
  int N_FRAMES = 100;
  if (argc > 1) {
    N_FRAMES = std::stoi(argv[1]);
  }

  std::cout << " Processing " << N_FRAMES << " Frames\n";
  Edrak::Frontend fe;
  Edrak::Map::SharedPtr map = std::make_shared<Edrak::Map>();
  fe.SetMap(map);

  std::vector<Edrak::CameraModel> camerasCalib =
      Edrak::ParseKITTICameras(imgs_path + "/calib.txt");
  Edrak::StereoCamera cam{camerasCalib[0], camerasCalib[1],
                          camerasCalib[1].pose.translation().norm()};
  fe.SetCamera(cam);

  // Create Viewer
  Edrak::Viewer::SharedPtr viewer = std::make_shared<Edrak::Viewer>();
  viewer->SetMap(map);
  fe.SetViewer(viewer);

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

  while (true) {
    usleep(5000);
    viewer->UpdateMap();
  }
  return 0;
}
