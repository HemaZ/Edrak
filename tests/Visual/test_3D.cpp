#include "../catch.hpp"
#include "Edrak/IO/Trajectory.hpp"
#include "Edrak/Types/Types.hpp"
#include "Edrak/Visual/3D.hpp"

TEST_CASE("Test visual trajectory", "visual") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string traj_path = data_dir + "IO/trajectory/estimated.txt";
  Edrak::TrajectoryD traj = Edrak::IO::LoadTrajectory(traj_path);
  // Edrak::Visual::DrawTrajectory(traj);
}