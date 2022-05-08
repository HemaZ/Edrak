#include "Edrak/VO/2D/PoseEstmation.hpp"
namespace Edrak {
namespace VO {
void PoseEstmation::Process(const cv::Mat &img) {
  if (!initialized_) {
    lastFrame_ = img;
    Features::ORB(lastFrame_, lastFrameKps_, lastFrameDescriptors_);
    initialized_ = true;
    return;
  }
  // Extract ORB features for current frame.
  Features::KeyPoints::KeyPoints kps;
  cv::Mat descriptors;
  Features::ORB(img, kps, descriptors);

  // Match current frame orb features with the prev frame.
  Features::Matches2D matches, filteredMatches;
  Features::FeaturesMatching(lastFrameDescriptors_, descriptors, matches);
  Features::FilterMatches(matches, filteredMatches, 0.3f);

  // Extract matched points.
  std::vector<cv::Point2f> currPoints, prevPoints;
  Features::MatchesToPoints(matches, lastFrameKps_, kps, prevPoints,
                            currPoints);

  cv::Mat essentialMatrix = cv::findEssentialMat(prevPoints, currPoints, k_.fx,
                                                 cv::Point2f(k_.cx, k_.cy));
  cv::Mat R, t;
  cv::recoverPose(essentialMatrix, prevPoints, currPoints, R, t, k_.fx,
                  cv::Point2f(k_.cx, k_.cy));

  Eigen::Matrix3d t_x;
  // clang-format off
  t_x << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
         t.at<double>(2, 0), 0, -t.at<double>(0, 0), 
         -t.at<double>(1, 0), t.at<double>(0, 0), 0.0;
  // clang-format on

  lastFrame_ = img;
  lastFrameKps_ = kps;
  lastFrameDescriptors_ = descriptors;
  Eigen::Matrix3d Reigen;
  Eigen::Vector3d teigen;
  cv::cv2eigen(R, Reigen);
  cv::cv2eigen(t, teigen);
  pose_ = Edrak::Types::SE3D(Reigen, teigen);

  // Check Epipolar constraint.
  for (size_t i = 0; i < prevPoints.size(); ++i) {
    Eigen::Vector3d y1, y2;
    y1 << k_.x(prevPoints[i].x), k_.x(prevPoints[i].y), 1;
    y2 << k_.x(currPoints[i].x), k_.x(currPoints[i].y), 1;
    Eigen::MatrixXd d = y2.transpose() * t_x * Reigen * y1;
    std::cout << "d[" << i << "] " << d << std::endl;
  }
}
} // namespace VO
} // namespace Edrak