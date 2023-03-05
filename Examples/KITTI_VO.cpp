#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/IO/PointCloud.hpp"
#include "Edrak/IO/Trajectory.hpp"
#include "Edrak/SLAM/VisualSLAM.hpp"
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
    imgs_path += "2011_09_26_drive_0064_sync";
  }

  std::cout << " Processing " << imgs_path << " Frames\n";
  std::cout << " Processing " << N_FRAMES << " Frames\n";

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

  Edrak::VisualSLAM slam(cam);

  auto settings = Edrak::SLAM::LoadSettings(imgs_path + "/settings.yaml");
  if (settings.has_value()) {
    slam.frontend->SetSettings(*settings);
  }

  Edrak::IO::MonoReader leftReader{imgs_path + "/image_0/*.png",
                                   Edrak::IO::ImageType::GRAY, false};
  Edrak::IO::MonoReader rightReader{imgs_path + "/image_1/*.png",
                                    Edrak::IO::ImageType::GRAY, false};
  std::string wait;
  Edrak::TrajectoryD trajectory;

  for (size_t i = 0; i < N_FRAMES; i++) {
    Edrak::StereoFrame::SharedPtr frame = Edrak::StereoFrame::CreateFrame();
    leftReader.NextFrame(frame->imgData);
    rightReader.NextFrame(frame->rightImgData);
    if (frame->imgData.size() == frame->rightImgData.size()) {
      slam.AddFrame(frame);
      auto state = slam.frontend->GetState();
      switch (state) {
      case Edrak::FrontendState::TRACKING:
        std::cout << "Tracking \n";
        break;
      case Edrak::FrontendState::LOST:
        std::cout << "LOST \n";
        break;
      case Edrak::FrontendState::INITIALIZING:
        std::cout << "Initializing \n";
        break;
      }
    } else {
      break;
    }
    // std::cout << " Frame " << i << " Pose " << fe.GetTwc().matrix() << '\n';
    trajectory.push_back(slam.frontend->GetTwc());
    // std::cin >> wait ;
  }

  Edrak::TrajectoryD KeyframesTrajbeforeBa;
  auto activeKfs = slam.map->GetAllKeyframes();
  for (const auto &kfPose : activeKfs) {
    KeyframesTrajbeforeBa.push_back(kfPose.second->Twc());
  }
  std::cout << "Writing KeyFrames Trajecotries File \n";
  Edrak::IO::ExportTrajectory(imgs_path + "/kfs_trajectory_before_ba.txt",
                              KeyframesTrajbeforeBa);
  std::cout << "Running BA \n";
  // slam.backend->OptimizeMap();
  std::cout << "Writing PLY File \n";
  Edrak::WritePLYFromLandmarks(slam.map->GetAllLandmarks(),
                               imgs_path + "/out.ply");
  Edrak::IO::ExportTrajectory(imgs_path + "/trajectory.txt", trajectory);

  Edrak::TrajectoryD KeyframesTraj;
  for (const auto &kfPose : slam.map->GetAllKeyframes()) {
    KeyframesTraj.push_back(kfPose.second->Twc());
  }
  slam.viewer->AddCurrentKFsTrajectory(KeyframesTraj);
  std::cout << "Writing KeyFrames Trajecotries File \n";
  Edrak::IO::ExportTrajectory(imgs_path + "/kfs_trajectory_after_ba.txt",
                              KeyframesTraj);

  while (true) {
    usleep(5000);
    slam.viewer->UpdateMap();
  }
  return 0;
}
