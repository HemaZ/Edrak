#include "../catch.hpp"
#include "Edrak/IO/Trajectory.hpp"
#include "Edrak/Images/RGBD.hpp"
#include "Edrak/Visual/3D.hpp"
#include <boost/format.hpp> // for formating strings

#include <iostream>
#include <opencv2/opencv.hpp>

TEST_CASE("RGBD", "RGBD") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  std::string imgs_path = data_dir + "Images/RGBD/";
  auto trajectory =
      Edrak::IO::LoadTrajectory(data_dir + "Images/RGBD/pose.txt");
  Edrak::Images::CameraMatD intrinsics{518.0, 519.0, 325.5, 253.5};
  Edrak::Types::PointCloudRGB pointcloud_all;
  for (int i = 0; i < 5; i++) {
    boost::format fmt("./%s/%d.%s");
    cv::Mat rgb =
        cv::imread(imgs_path + (fmt % "color" % (i + 1) % "png").str());
    cv::Mat depth =
        cv::imread(imgs_path + (fmt % "depth" % (i + 1) % "pgm").str(), -1);
    auto pointcloud = Edrak::Images::RGBDToPointCloud(rgb, depth, intrinsics,
                                                      1000, trajectory[0]);
    pointcloud_all.insert(pointcloud_all.end(), pointcloud.begin(),
                          pointcloud.end());
  }
  Edrak::Visual::DrawPointCloud(pointcloud_all);
}