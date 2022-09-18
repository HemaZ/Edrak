#include "../catch.hpp"
#include "Edrak/SLAM/Frontend.hpp"
#include <iostream>

TEST_CASE("Test StereoInit", "Frontend::StereoInit") {
  Edrak::Frontend fe;
  REQUIRE_FALSE(fe.AddFrame(nullptr));
}