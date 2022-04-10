#include "../catch.hpp"
#include "Edrak/Benchamrk/Timer.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

TEST_CASE("OpenCv_Features", "OrbFeatures") {
  std::string data_dir = EDRAK_TEST_DATA_DIR;
  cv::Mat im1 = cv::imread(data_dir + "Images/Orb/0000000026.png");
  cv::Mat im2 = cv::imread(data_dir + "Images/Orb/0000000027.png");

  std::vector<cv::KeyPoint> keyPoints1, keyPoints2;
  cv::Mat descriptors1, descriptors2;

  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");

  Edrak::Benchmark::Timer tOrbFeatures("Orb Featueres");
  // Detect Keypoints
  Edrak::Benchmark::Timer tKeypoints("ORB KeyPoints");
  detector->detect(im1, keyPoints1);
  detector->detect(im2, keyPoints2);
  tKeypoints.Stop();

  // Compute BRIEF Descriptors
  Edrak::Benchmark::Timer tDescriptors("ORB Descriptors");
  descriptor->compute(im1, keyPoints1, descriptors1);
  descriptor->compute(im2, keyPoints2, descriptors2);
  tDescriptors.Stop();

  tOrbFeatures.Stop();

  // Draw KeyPoints
  cv::Mat outIm1;
  cv::drawKeypoints(im1, keyPoints1, outIm1);
  //   cv::imshow("Orb Features", outIm1);
  //   cv::waitKey(0);

  // Match Keypoints
  std::vector<cv::DMatch> matches;
  Edrak::Benchmark::Timer tMatchDescriptors("Matching ORB Descriptors");
  matcher->match(descriptors1, descriptors2, matches);
  tMatchDescriptors.Stop();

  const auto [min, max] = std::minmax_element(
      matches.begin(), matches.end(),
      [](const cv::DMatch &first, const cv::DMatch &second) {
        return first.distance < second.distance;
      });

  std::cout << "Max Distance " << max->distance << std::endl;
  std::cout << "Min Distance " << min->distance << std::endl;

  float filerDist = (max->distance - min->distance) * 0.30;
  // Filter Bad matches
  std::vector<cv::DMatch> filteredMatches;
  for (size_t i = 0; i < descriptors1.rows; i++) {
    if (matches[i].distance < filerDist)
      filteredMatches.push_back(matches[i]);
  }

  // Draw Matches
  cv::Mat imallMatches, imfilteredMatches;
  cv::drawMatches(im1, keyPoints1, im2, keyPoints2, matches, imallMatches);
  cv::drawMatches(im1, keyPoints1, im2, keyPoints2, filteredMatches,
                  imfilteredMatches);
  // cv::imshow("all matches", imallMatches);
  // cv::imshow("Filtered matches", imfilteredMatches);
  // cv::waitKey(0);
}