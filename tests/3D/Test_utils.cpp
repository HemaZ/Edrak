#include "../catch.hpp"
#include "Edrak/3D/Utils.hpp"
#include <iostream>
TEST_CASE("3D_Points3DCOM", "Points3DCOM") {
  // clang-format off
    Edrak::Points3D pts1(3,3);
    pts1 << 1,4,3,
            1,1,3,
            1,1,3;
  // clang-format on
  Edrak::Point3D pCom = Edrak::Points3DCOM(pts1);
  REQUIRE(pCom(0) == 1.0);
  REQUIRE(pCom(1) == 2.0);
  REQUIRE(pCom(2) == 3.0);
}

TEST_CASE("3D_RemoveCOMPoints3D", "RemoveCOMPoints3D") {
  // clang-format off
    Edrak::Points3D pts1(3,3);
    pts1 << 1,4,3,
            0,1,2,
            2,1,4;
  // clang-format on
  Edrak::RemoveCOMPoints3D(pts1);
  std::cout << pts1 << std::endl;
  //   REQUIRE(pts1.row(0) == Edrak::Point3D::Zero().eval());
  //   REQUIRE(pts1.row(1) == Edrak::Point3D::Zero().eval());
  //   REQUIRE(pts1.row(2) == Edrak::Point3D::Zero().eval());
}