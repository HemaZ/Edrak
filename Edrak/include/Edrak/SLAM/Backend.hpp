#ifndef __EDRAK_SLAM_BACKEND_H__
#define __EDRAK_SLAM_BACKEND_H__
#include "Edrak/Images/Features.hpp"
#include "Edrak/Images/Frame.hpp"
#include "Edrak/SLAM/Map.hpp"
namespace Edrak {
class Backend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using SharedPtr = std::shared_ptr<Backend>;

  void UpdateMap();
};

} // namespace Edrak

#endif