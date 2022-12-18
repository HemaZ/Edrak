#include "Edrak/Benchamrk/Timer.hpp"
#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/VO/2D/PoseEstmation.hpp"
#include "Edrak/Visual/3D.hpp"

int main(int argc, char const *argv[]) {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "KITTI/VO/16";

  std::vector<Edrak::CameraModel> camerasCalib =
      Edrak::ParseKITTICameras(imgs_path + "/calib.txt");
  Edrak::StereoCamera cam{camerasCalib[0], camerasCalib[1],
                          camerasCalib[1].pose.translation().norm()};

  Edrak::VO::PoseEstmation pe(camerasCalib[0].calibration);

  Edrak::IO::MonoReader reader{imgs_path + "/image_0/*.png",
                               Edrak::IO::ImageType::GRAY, false};
  cv::Mat frame;
  Edrak::TrajectoryD trajectory;
  for (size_t i = 0; i < 10; i++) {
    reader.NextFrame(frame);
    pe.Process(frame);
    std::cout << "Pose" << i << pe.Pose().matrix() << std::endl;
    trajectory.push_back(pe.Pose());
  }
  Edrak::Visual::DrawTrajectory(trajectory);
  return 0;
}
