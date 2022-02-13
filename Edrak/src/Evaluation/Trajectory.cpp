#include "Edrak/Evaluation/Trajectory.hpp"
#include "Edrak/Exceptions/Exceptions.hpp"

namespace Edrak {
namespace Evaluation {
double RMSE(const Edrak::Types::TrajectoryD &gt,
            const Edrak::Types::TrajectoryD &est) {
  if (gt.size() != est.size()) {
    throw Edrak::Exceptions::WrongSize(
        "The two trajectories sizes don't match!");
  }
  double total_error = 0.0;
  for (size_t i = 0; i < gt.size(); i++) {
    double error = (gt[i].inverse() * est[i]).log().norm();
    total_error += error * error;
  }
  total_error /= static_cast<double>(gt.size());
  total_error = sqrt(total_error);
  return total_error;
}
} // namespace Evaluation

} // namespace Edrak
