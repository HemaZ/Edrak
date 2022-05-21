#ifndef EDRAK_INCLUDE_EDRAK_3D_UTILS
#define EDRAK_INCLUDE_EDRAK_3D_UTILS
#include "Edrak/Types/Types.hpp"
namespace Edrak {
/**
 * @brief Computes center of mass for a 3D Points set.
 *
 * @param pts 3D Points set.
 * @return Edrak::Point3D Center of mass.
 */
inline Edrak::Point3D Points3DCOM(const Edrak::Points3D &pts) {
  Edrak::Point3D pCOM;
  //   pCOM << pts.col(0).sum(), pts.col(1).sum(), pts.col(2).sum();
  pCOM = pts.colwise().sum() / pts.rows();
  return pCOM;
}

/**
 * @brief
 *
 * @param pts
 * @param pCOM
 */
inline void RemoveCOMPoints3D(Edrak::Points3D &pts,
                              const Edrak::Point3D &pCOM) {
  pts.rowwise() -= pCOM.transpose();
}

/**
 * @brief
 *
 * @param pts
 */
inline void RemoveCOMPoints3D(Edrak::Points3D &pts) {
  Edrak::Point3D pCom = Points3DCOM(pts);
  RemoveCOMPoints3D(pts, pCom);
}

} // namespace Edrak

#endif /* EDRAK_INCLUDE_EDRAK_3D_UTILS */
