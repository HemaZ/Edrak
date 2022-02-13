#ifndef __EDRAK_EVAL_TRAJECTORY_H__
#define __EDRAK_EVAL_TRAJECTORY_H__
#include "Edrak/Types/Transformation.hpp"
#include <sophus/se3.hpp>

namespace Edrak {
namespace Evaluation {
double RMSE(const Edrak::Types::TrajectoryD &gt,
            const Edrak::Types::TrajectoryD &est);
} // namespace Evaluation

} // namespace Edrak

#endif // __EDRAK_EVAL_TRAJECTORY_H__