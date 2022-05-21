#include "../catch.hpp"
#include "Edrak/Types/Types.hpp"
#include <iostream>
TEST_CASE("Test Quaterion initialization list", "Quaterion") {
  Edrak::QuatD q{0, 1, 0, 0};
  REQUIRE(q.w() == 0);
  REQUIRE(q.x() == 1);
  REQUIRE(q.y() == 0);
  REQUIRE(q.z() == 0);
}

TEST_CASE("Test Quaterion non unit", "Quaterion") {
  Edrak::QuatD q{0, 1, 1, 0};
  REQUIRE(q.coeffs() == Eigen::Vector4d{1, 1, 0, 0});
}

TEST_CASE("Test Quaterion from Transformation Matrix", "Quaterion") {
  Edrak::TransMatD tf_mat = Edrak::TransMatD::Identity();
  Edrak::QuatD q{tf_mat};
  REQUIRE(q.coeffs() == Eigen::Vector4d{0, 0, 0, 1});
}

TEST_CASE("Test rpy to quat", "Quaterion") {
  Edrak::QuatD q;
  Edrak::RPYToQuat<double>(0, 0, 0, q);
  REQUIRE(q.coeffs() == Eigen::Vector4d{0, 0, 0, 1});
}

TEST_CASE("Test 2 rpy to quat", "Quaterion") {
  Edrak::QuatF q;
  Edrak::RPYToQuat<float>(3.14, 0, 0, q);
  REQUIRE(q.x() == Approx(0.9999997).margin(0.00001));
  REQUIRE(q.y() == Approx(0));
  REQUIRE(q.z() == Approx(0));
  REQUIRE(q.w() == Approx(0.0007963).margin(0.00001));
}