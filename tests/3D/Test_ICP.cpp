#include "../catch.hpp"
#include "Edrak/3D/ICP.hpp"
#include "Edrak/Images/Calibration.hpp"
#include "Edrak/Images/Features.hpp"
#include "Edrak/Visual/3D.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
TEST_CASE("ICP", "ICP") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  cv::Mat img1 = cv::imread(data_dir + "3D/ICP/1.png", cv::IMREAD_COLOR);
  cv::Mat img2 = cv::imread(data_dir + "3D/ICP/2.png", cv::IMREAD_COLOR);
  cv::Mat img1_depth =
      cv::imread(data_dir + "3D/ICP/1_depth.png", cv::IMREAD_UNCHANGED);
  cv::Mat img2_depth =
      cv::imread(data_dir + "3D/ICP/2_depth.png", cv::IMREAD_UNCHANGED);
  Eigen::Matrix3d mat;
  mat << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
  Edrak::Images::CameraMatD K = Edrak::Images::CameraMatD::FromMat(mat);

  std::vector<cv::KeyPoint> keyPoints1, keyPoints2;
  cv::Mat descriptors1, descriptors2;
  std::vector<cv::DMatch> matches;
  Edrak::Images::Features::ExtractORBMatches(img1, img2, keyPoints1, keyPoints2,
                                             matches);

  std::vector<cv::DMatch> filteredMatches;
  Edrak::Images::Features::FilterMatches(matches, filteredMatches, 0.30);
  Edrak::Points3D pts1(filteredMatches.size(), 3),
      pts2(filteredMatches.size(), 3);

  cv::Mat imfilteredMatches;
  cv::drawMatches(img1, keyPoints1, img2, keyPoints2, filteredMatches,
                  imfilteredMatches);
  cv::imshow("Filtered matches", imfilteredMatches);
  cv::waitKey(0);

  size_t j = 0;
  for (size_t i = 0; i < filteredMatches.size(); i++) {
    auto match = filteredMatches[i];

    cv::Point2f p1 = keyPoints1[match.queryIdx].pt;
    cv::Point2f p2 = keyPoints2[match.trainIdx].pt;

    ushort d1s = img1_depth.ptr<unsigned short>(int(p1.y))[int(p1.x)];
    ushort d2s = img2_depth.ptr<unsigned short>(int(p2.y))[int(p2.x)];
    if (d1s == 0 || d2s == 0)
      continue;

    float d1 = d1s / 5000.0f;
    float d2 = d2s / 5000.0f;

    pts1.row(j) << K.x(p1.x, d1), K.y(p1.y, d1), d1;
    pts2.row(j++) << K.x(p2.x, d2), K.y(p2.y, d2), d2;
  }
  pts1.conservativeResize(j, 3);
  pts2.conservativeResize(j, 3);

  std::ofstream file("pts1.txt");
  file << pts1;
  file.close();
  std::ofstream file2("pts2.txt");
  file2 << pts2;
  file2.close();
  pts1.conservativeResize(j, 3);
  Edrak::SE3D T = Edrak::ICP(pts1, pts2);
  std::cout << T.rotationMatrix() << std::endl;
  std::cout << T.translation() << std::endl;
  Edrak::Visual::DrawTrajectory(
      {Edrak::SE3D{Edrak::SO3D{Eigen::Matrix3d::Identity()},
                   Eigen::Vector3d::Zero()},
       T});
}