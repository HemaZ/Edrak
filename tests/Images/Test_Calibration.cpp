#include "../catch.hpp"
#include "Edrak/Images/Calibration.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

TEST_CASE("Camera Matrix", "CameraMatrix") {
  Edrak::Images::CameraMatF camMat{458.645, 457.296, 367.215, 248.375};
  REQUIRE(camMat.fx == Approx(458.645).margin(0.00001));

  Edrak::Images::CameraMatrix<double> camMatD{458.645, 457.296, 367.215,
                                              248.375};
  REQUIRE(camMatD.fx == Approx(458.645).margin(0.00001));
}

TEST_CASE("Camera Matrix from Matrix", "CameraMatrix") {
  // from KITTI dataset
  // 9.812178e+02 0.000000e+00 6.900000e+02
  // 0.000000e+00 9.758994e+02 2.471364e+02 0.000000e+00
  // 0.000000e+00 1.000000e+00

  Eigen::Matrix3d mat;
  mat << 9.812178e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00, 9.758994e+02,
      2.471364e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00;

  Edrak::Images::CameraMatD camMatfromMat =
      Edrak::Images::CameraMatD::FromMat(mat);
  Edrak::Images::CameraMatD camMat{9.812178e+02, 9.758994e+02, 6.900000e+02,
                                   2.471364e+02};

  REQUIRE(camMatfromMat.fx == Approx(camMat.fx).margin(0.00001));
  REQUIRE(camMatfromMat.fy == Approx(camMat.fy).margin(0.00001));
  REQUIRE(camMatfromMat.cx == Approx(camMat.cx).margin(0.00001));
  REQUIRE(camMatfromMat.cy == Approx(camMat.cy).margin(0.00001));
}

TEST_CASE("Undistort", "Undistort") {
  Edrak::Images::CameraMatD camMat{458.645, 457.296, 367.215, 248.375};
  Edrak::Images::RadTanCoeffsD RadTan{-0.28340811, 0.07395907, 0.00019359,
                                      1.76187114e-05};
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string img_path = data_dir + "Images/Calibration/distorted.png";
  cv::Mat input = cv::imread(img_path, 0);
  cv::Mat undistortedImg = Edrak::Images::Undistort(input, camMat, RadTan);
  //   cv::imshow("distorted", input);
  //   cv::imshow("undistorted", undistortedImg);
  //   cv::waitKey();
}

TEST_CASE("UndistortRGB", "Undistort") {
  // from KITTI dataset
  // 9.812178e+02 0.000000e+00 6.900000e+02
  // 0.000000e+00 9.758994e+02 2.471364e+02 0.000000e+00
  // 0.000000e+00 1.000000e+00
  Edrak::Images::CameraMatD camMat{9.812178e+02, 9.758994e+02, 6.900000e+02,
                                   2.471364e+02};
  Edrak::Images::RadTanCoeffsD RadTan{-3.791375e-01, 2.148119e-01, 1.227094e-03,
                                      2.343833e-03};
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string img_path = data_dir + "Images/Calibration/distortedrgb.png";
  cv::Mat input = cv::imread(img_path, 1);
  cv::Mat undistortedImg = Edrak::Images::Undistort(input, camMat, RadTan);
  //   cv::imshow("distorted", input);
  //   cv::imshow("undistorted", undistortedImg);
  //   cv::waitKey();
}