#include "../catch.hpp"
#include "Edrak/Types/Transformation.hpp"
#include "Edrak/Visual/3D.hpp"

TEST_CASE("Test visual trajectory", "visual") {
  Edrak::Types::TrajectoryD trajectory;
  Edrak::Types::SE3D pose{Edrak::Types::QuatD{1, 0, 0, 0}};
  pose.pretranslate(Eigen::Vector3d{1, 0, 0});
  trajectory.push_back(pose);
  Edrak::Types::SE3D pose2{Edrak::Types::QuatD{1, 0, 0, 0}};
  pose2.pretranslate(Eigen::Vector3d{2, 0, 0});
  trajectory.push_back(pose2);
  Edrak::Types::SE3D pose3{Edrak::Types::QuatD{1, 0, 0, 0}};
  pose3.pretranslate(Eigen::Vector3d{3, 0, 0});
  trajectory.push_back(pose3);
  Edrak::Visual::DrawTrajectory(trajectory);
}