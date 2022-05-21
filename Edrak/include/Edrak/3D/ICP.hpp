#ifndef EDRAK_INCLUDE_EDRAK_3D_ICP
#define EDRAK_INCLUDE_EDRAK_3D_ICP
#include "Edrak/Types/Types.hpp"
#include <numeric>

namespace Edrak {
/**
 * @brief Performing the Iterative closest point (ICP) algorithm on two sets of
 * 3D points.
 *
 * @param pts1 First 3D Points set.
 * @param pts2 Second 3D Points set.
 * @param pose The output Transformation which transform the second set to the
 * first set.
 */
Edrak::SE3D ICP(const Points3D &pts1, const Points3D &pts2);
} // namespace Edrak

#endif /* EDRAK_INCLUDE_EDRAK_3D_ICP */
