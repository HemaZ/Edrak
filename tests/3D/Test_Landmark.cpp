#include "../catch.hpp"
#include "Edrak/3D/Landmark.hpp"

TEST_CASE("Landmark", "Landmark::CreateLandmark") {
  Edrak::Landmark::SharedPtr l0 = Edrak::Landmark::CreateLandmark();
  Edrak::Landmark::SharedPtr l1 = Edrak::Landmark::CreateLandmark();
  REQUIRE(l0->id == 0);
  REQUIRE(l1->id == 1);
}