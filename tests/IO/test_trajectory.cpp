#include "../catch.hpp"
#include "Edrak/IO/Trajectory.hpp"
#include <cstdlib>

TEST_CASE("test trajectory loading", "IO") {

  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string traj_path = data_dir + "IO/trajectory/estimated.txt";
  Edrak::Types::TrajectoryD traj = Edrak::IO::LoadTrajectory(traj_path);
  REQUIRE(traj.size() == 612);
}