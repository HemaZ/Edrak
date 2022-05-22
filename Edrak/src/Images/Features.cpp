#include "Edrak/Images/Features.hpp"
#include "Edrak/Exceptions/Exceptions.hpp"

namespace Edrak {
namespace Images {
namespace Features {

namespace KeyPoints {
void FAST(const cv::Mat &img, KeyPoints &kps, int threshold,
          bool non_max_suppresion) {
  cv::FAST(img, kps, threshold, non_max_suppresion);
}

} // namespace KeyPoints

void ORB(const cv::Mat &img, KeyPoints::KeyPoints &kps, cv::Mat &descriptors) {
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  detector->detect(img, kps);
  descriptor->compute(img, kps, descriptors);
}

void ExtractORBMatches(const cv::Mat &img1, const cv::Mat &img2,
                       KeyPoints::KeyPoints &keyPoints1,
                       KeyPoints::KeyPoints &keyPoints2, Matches2D &matches) {
  cv::Mat descriptors1, descriptors2;
  ORB(img1, keyPoints1, descriptors1);
  ORB(img2, keyPoints2, descriptors2);
  FeaturesMatching(descriptors1, descriptors2, matches);
}

void FeaturesMatching(const cv::Mat &descriptors1, const cv::Mat &descriptors2,
                      Matches2D &matches) {
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors1, descriptors2, matches);
}

void FilterMatches(const Matches2D &matches, Matches2D &filteredMatches,
                   float ratio) {
  const auto [min, max] = std::minmax_element(
      matches.begin(), matches.end(),
      [](const cv::DMatch &first, const cv::DMatch &second) {
        return first.distance < second.distance;
      });

  // std::cout << "Max Distance " << max->distance << std::endl;
  // std::cout << "Min Distance " << min->distance << std::endl;
  float filerDist = (max->distance - min->distance) * ratio;
  // Filter Bad matches
  for (const auto &match : matches) {
    if (match.distance < filerDist)
      filteredMatches.push_back(match);
  }
}

} // namespace Features

} // namespace Images
} // namespace Edrak