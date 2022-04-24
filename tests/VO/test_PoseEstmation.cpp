#include "../catch.hpp"
#include "Edrak/Benchamrk/Timer.hpp"
#include "Edrak/IO/MonoReader.hpp"
#include "Edrak/VO/2D/PoseEstmation.hpp"
#include "Edrak/Visual/3D.hpp"

TEST_CASE("TEST_POSE_ESTMATION", "VO") {
  Edrak::Images::CameraMatD camMat{9.812178e+02, 9.758994e+02, 6.900000e+02,
                                   2.471364e+02};
  Edrak::VO::PoseEstmation pe(camMat);

  std::string data_dir = EDRAK_TEST_DATA_DIR;

  Edrak::IO::MonoReader reader(data_dir + "VO/*.png");
  cv::Mat frame;
  Edrak::Types::TrajectoryD trajectory;
  for (size_t i = 0; i < 10; i++) {
    reader.NextFrame(frame);
    pe.Process(frame);
    std::cout << "Pose" << i << pe.Pose().matrix() << std::endl;
    trajectory.push_back(pe.Pose());
  }
  Edrak::Visual::DrawTrajectory(trajectory);
}