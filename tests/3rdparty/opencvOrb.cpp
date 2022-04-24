#include "../catch.hpp"
#include "Edrak/Benchamrk/Timer.hpp"
#include "Edrak/Images/Features.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

TEST_CASE("OpenCv_Features", "OrbFeatures") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  cv::Mat im1 = cv::imread(data_dir + "Images/Orb/0000000026.png");
  cv::Mat im2 = cv::imread(data_dir + "Images/Orb/0000000027.png");

  std::vector<cv::KeyPoint> keyPoints1, keyPoints2;
  cv::Mat descriptors1, descriptors2;
  std::vector<cv::DMatch> matches;

  Edrak::Benchmark::Timer tOrbFeatures("Orb Featueres");
  Edrak::Images::Features::ExtractORBMatches(im1, im2, keyPoints1, keyPoints2,
                                             matches);
  tOrbFeatures.Stop();

  std::vector<cv::DMatch> filteredMatches;
  Edrak::Images::Features::FilterMatches(matches, filteredMatches, 0.30);

  // Draw Matches
  cv::Mat imallMatches, imfilteredMatches;
  cv::drawMatches(im1, keyPoints1, im2, keyPoints2, matches, imallMatches);
  cv::drawMatches(im1, keyPoints1, im2, keyPoints2, filteredMatches,
                  imfilteredMatches);
  // cv::imshow("all matches", imallMatches);
  // cv::imshow("Filtered matches", imfilteredMatches);
  // cv::waitKey(0);
}