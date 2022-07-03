#ifndef EXAMPLES_BA_SNAVELYREPROJECTIONERROR
#define EXAMPLES_BA_SNAVELYREPROJECTIONERROR
#include "ceres/ceres.h"
#include "rotation.h"
#include <iostream>

class SnavelyReprojectionError {
public:
  SnavelyReprojectionError(double observation_x, double observation_y)
      : observed_x(observation_x), observed_y(observation_y) {}
  template <typename T>
  bool operator()(const T *const camera, const T *const point,
                  T *residual) const {
    T predictions[2];
    CamProjectionWithDistortion<T>(camera, point, predictions);
    residual[0] = T(observed_x) - predictions[0];
    residual[1] = T(observed_y) - predictions[1];
    return true;
  }
  template <typename T>
  void CamProjectionWithDistortion(const T *camera, const T *point,
                                   T *predictions) const {
    T p[3];
    AngleAxisRotatePoint(camera, point, p);

    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion
    const T &l1 = camera[7];
    const T &l2 = camera[8];

    T r2 = xp * xp + yp * yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2);

    const T &focal = camera[6];
    predictions[0] = focal * distortion * xp;
    predictions[1] = focal * distortion * yp;
  }
  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }

private:
  double observed_x;
  double observed_y;
};
#endif /* EXAMPLES_BA_SNAVELYREPROJECTIONERROR */
