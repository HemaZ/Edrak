#include "../catch.hpp"
#include "Edrak/Benchamrk/Timer.hpp"
#include "Edrak/Images/Frame.hpp"
#include <iostream>

TEST_CASE("CreateFrame", "Frame::CreateFrame") {
  Edrak::Frame::SharedPtr frame0 = Edrak::Frame::CreateFrame();
  Edrak::Frame::SharedPtr frame1 = Edrak::Frame::CreateFrame();
  REQUIRE(frame0->frameId == 0);
  REQUIRE(frame1->frameId == 1);
}