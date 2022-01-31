#include "../catch.hpp"
#include "Edrak/Images/TemplateMatching.hpp"
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
TEST_CASE("Template Dimensions Validation", "[Edrak::Images::matchTemplate()") {

  cv::Mat img(256, 256, CV_8UC1);
  cv::Mat templ(512, 512, CV_8UC1);
  cv::Mat res;
  REQUIRE_THROWS_AS(Edrak::Images::matchTemplate(
                        img, templ, res,
                        Edrak::Images::TemplateMatchingMethod::CC_NORMALIZED),
                    Edrak::Exceptions::InvalidDimensions);
  templ.create(64, 512, CV_8UC1);
  REQUIRE_THROWS_AS(Edrak::Images::matchTemplate(
                        img, templ, res,
                        Edrak::Images::TemplateMatchingMethod::CC_NORMALIZED),
                    Edrak::Exceptions::InvalidDimensions);
  templ.create(512, 64, CV_8UC1);
  REQUIRE_THROWS_AS(Edrak::Images::matchTemplate(
                        img, templ, res,
                        Edrak::Images::TemplateMatchingMethod::CC_NORMALIZED),
                    Edrak::Exceptions::InvalidDimensions);
  templ.create(64, 64, CV_8UC1);
  REQUIRE_NOTHROW(Edrak::Images::matchTemplate(
      img, templ, res, Edrak::Images::TemplateMatchingMethod::CC_NORMALIZED));

  templ.create(256, 256, CV_8UC1);
  REQUIRE_NOTHROW(Edrak::Images::matchTemplate(
      img, templ, res, Edrak::Images::TemplateMatchingMethod::CC_NORMALIZED));
}

TEST_CASE("Results Dimensions Validation", "[Edrak::Images::matchTemplate()") {

  cv::Mat img(512, 512, CV_8UC1);
  cv::Mat templ(64, 64, CV_8UC1);
  cv::Mat res;
  Edrak::Images::matchTemplate(
      img, templ, res, Edrak::Images::TemplateMatchingMethod::CC_NORMALIZED);
  REQUIRE(res.cols == 449);
  REQUIRE(res.rows == 449);
}

TEST_CASE("Filter2D", "[Edrak::Images::matchTemplate()") {

  cv::Mat img = cv::imread("../tests/2011_09_26/2011_09_26_drive_0002_sync/"
                           "image_00/data/0000000000.png",
                           cv::IMREAD_GRAYSCALE);
  cv::Mat templ =
      cv::imread("../tests/2011_09_26/2011_09_26_drive_0002_sync/image_00/"
                 "0000000000_template.png",
                 cv::IMREAD_GRAYSCALE);

  cv::Mat res;
  Edrak::Images::matchTemplate(
      img, templ, res, Edrak::Images::TemplateMatchingMethod::CC_NORMALIZED);
  cv::imshow("Display Image", res);

  cv::waitKey(0);
}