#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/IO/PointCloud.hpp"
#include "Edrak/IO/Trajectory.hpp"
#include "Edrak/SLAM/VisualSLAM.hpp"
#include "Edrak/Visual/3D.hpp"
#include "argh.h"
#include <memory>
#include <unistd.h>

int main(int argc, char const *argv[]) {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "KITTI/";

  argh::parser cmdl(argv);
  bool headless = false;
  bool runBA = false;
  int seqEnd = -1;

  if (cmdl(1)) {
    if (cmdl[1][0] != '/') {
      imgs_path += cmdl[1];
    } else {
      imgs_path = cmdl[1];
    }
  } else {
    std::cerr << "No KITTI sequence is provided. Exiting !";
    return -1;
  }

  cmdl(2, -1) >> seqEnd;

  if (cmdl[{"-s", "--headless"}]) {
    headless = true;
  }

  if (cmdl[{"-ba", "--run-ba"}]) {
    runBA = true;
  }

  std::cout << " Processing " << imgs_path << " Frames\n";

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

  Edrak::VisualSLAM slam(cam, headless);

  auto settings = Edrak::SLAM::LoadSettings(imgs_path + "/settings.yaml");
  if (settings.has_value()) {
    std::cout << "Settings file loaded from path "
              << imgs_path + "/settings.yaml\n";
    slam.frontend->SetSettings(*settings);
  }

  Edrak::IO::MonoReader leftReader{imgs_path + "/image_0/*.png",
                                   Edrak::IO::ImageType::GRAY, false};
  Edrak::IO::MonoReader rightReader{imgs_path + "/image_1/*.png",
                                    Edrak::IO::ImageType::GRAY, false};
  std::string wait;
  Edrak::TrajectoryD trajectory;
  while (true) {
    // Create a stereo frame
    Edrak::StereoFrame::SharedPtr frame = Edrak::StereoFrame::CreateFrame();
    if (!leftReader.NextFrame(frame->imgData) ||
        !rightReader.NextFrame(frame->rightImgData)) {
      break;
    }
    if (frame->imgData.size() != frame->rightImgData.size()) {
      std::cerr << "Left frame resolution is not equal to the right frame "
                   "resolution\n";
      break;
    }
    // Add the frame to the system
    slam.AddFrame(frame);
    // Get the tracking state
    auto state = slam.frontend->GetState();
    std::cout << "#" << frame->frameId << ": ";
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

    // std::cout << " Frame " << i << " Pose " << fe.GetTwc().matrix() << '\n';
    trajectory.push_back(slam.frontend->GetTwc());
    // std::cin >> wait ;
    if (frame->frameId == seqEnd) {
      break;
    }
  }

  Edrak::TrajectoryD KeyframesTrajbeforeBa;
  auto activeKfs = slam.map->GetAllKeyframes();
  for (const auto &kfPose : activeKfs) {
    KeyframesTrajbeforeBa.push_back(kfPose.second->Twc());
  }
  std::cout << "Writing KeyFrames Trajectories File \n";
  Edrak::IO::ExportTrajectory(imgs_path + "/kfs_trajectory_before_ba.txt",
                              KeyframesTrajbeforeBa);
  std::cout << "Writing PLY File \n";
  Edrak::WritePLYFromLandmarks(slam.map->GetAllLandmarks(),
                               imgs_path + "/out.ply");
  if (runBA) {
    std::cout << "Running BA \n";
    slam.backend->OptimizeMap();
    Edrak::TrajectoryD KeyframesTraj;
    for (const auto &kfPose : slam.map->GetAllKeyframes()) {
      KeyframesTraj.push_back(kfPose.second->Twc());
    }
    slam.viewer->AddCurrentKFsTrajectory(KeyframesTraj);
    std::cout << "Writing KeyFrames Trajectories File \n";
    Edrak::IO::ExportTrajectory(imgs_path + "/kfs_trajectory_after_ba.txt",
                                KeyframesTraj);
  }

  while (true && !headless) {
    usleep(5000);
    slam.viewer->UpdateMap();
  }
  return 0;
}
