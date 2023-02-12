#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/SLAM/Frontend.hpp"
#include "Edrak/SLAM/Viewer.hpp"
#include "Edrak/Visual/3D.hpp"
#include <memory>
#include <unistd.h>
int main(int argc, char const *argv[]) {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "KITTI/";
  int N_FRAMES = 100;
  if (argc > 1) {
    N_FRAMES = std::stoi(argv[1]);
  }
  if (argc > 2) {
    imgs_path += argv[2];
  } else {
    imgs_path += "2011_09_26_drive_0093_sync";
  }

  std::cout << " Processing " << imgs_path << " Frames\n";
  std::cout << " Processing " << N_FRAMES << " Frames\n";
  Edrak::Frontend fe;
  Edrak::Map::SharedPtr map = std::make_shared<Edrak::Map>();
  fe.SetMap(map);

  // std::vector<Edrak::CameraModel> camerasCalib =
  // Edrak::ParseKITTICameras(imgs_path + "/calib.txt");
  Edrak::CameraModel leftCamCalib(721.5377, 721.5377, 609.5593, 172.8540,
                                  Sophus::SE3d());
  Eigen::Vector3d t;
  t << -5.370000e-01, 4.822061e-03, -1.252488e-02;
  Edrak::CameraModel rightCamCalib(721.5377, 721.5377, 609.5593, 172.8540,
                                   Sophus::SE3d(Sophus::SO3d(), t));

  Edrak::StereoCamera cam{leftCamCalib, rightCamCalib,
                          rightCamCalib.pose.translation().norm()};
  fe.SetCamera(cam);

  // Create Viewer
  Edrak::Viewer::SharedPtr viewer = std::make_shared<Edrak::Viewer>();
  viewer->SetMap(map);
  fe.SetViewer(viewer);

  Edrak::IO::MonoReader leftReader{imgs_path + "/image_00/data/*.png",
                                   Edrak::IO::ImageType::GRAY, false};
  Edrak::IO::MonoReader rightReader{imgs_path + "/image_01/data/*.png",
                                    Edrak::IO::ImageType::GRAY, false};
  std::string wait;
  Edrak::TrajectoryD trajectory;

  for (size_t i = 0; i < N_FRAMES; i++) {
    Edrak::StereoFrame::SharedPtr frame = Edrak::StereoFrame::CreateFrame();
    leftReader.NextFrame(frame->imgData);
    rightReader.NextFrame(frame->rightImgData);
    if (frame->imgData.size() == frame->rightImgData.size()) {
      fe.AddFrame(frame);
    } else {
      break;
    }
    // std::cout << " Frame " << i << " Pose " << fe.GetTwc().matrix() << '\n';
    trajectory.push_back(fe.GetTwc());
    // std::cin >> wait ;
  }

  while (true) {
    usleep(5000);
    viewer->UpdateMap();
  }
  return 0;
}
