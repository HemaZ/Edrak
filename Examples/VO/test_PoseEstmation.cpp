#include "Edrak/Benchamrk/Timer.hpp"
#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/VO/2D/PoseEstmation.hpp"
#include "Edrak/Visual/3D.hpp"

int main(int argc, char const *argv[]) {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "KITTI/2011_09_26_drive_0064_sync";
  Edrak::CameraModel leftCamCalib(721.5377,721.5377,609.5593, 172.8540, Sophus::SE3d());
  Edrak::VO::PoseEstmation pe(leftCamCalib.calibration);

  Edrak::IO::MonoReader reader{imgs_path + "/image_00/data/*.png",
                               Edrak::IO::ImageType::GRAY, false};
  cv::Mat frame;
  Edrak::TrajectoryD trajectory;
  Edrak::SE3D lastPose;
  for (size_t i = 0; i < 500; i++) {
    reader.NextFrame(frame);
    pe.Process(frame);
    std::cout << "Pose" << i << pe.Pose().matrix() << std::endl;
    Edrak::SE3D Twc = lastPose * pe.Pose();
    trajectory.push_back(Twc);
    lastPose = Twc;
  }
  Edrak::Visual::DrawTrajectory(trajectory);
  return 0;
}
