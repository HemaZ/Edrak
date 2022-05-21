#include "Edrak/3D/ICP.hpp"
#include "Edrak/3D/Utils.hpp"
#include <fstream>
namespace Edrak {
Edrak::SE3D ICP(const Edrak::Points3D &pts1, const Edrak::Points3D &pts2) {

  // Compute points Centroid
  Edrak::Point3D p1 = Points3DCOM(pts1);
  Edrak::Point3D p2 = Points3DCOM(pts2);

  // Remove points Centroid.
  Edrak::Points3D q1 = pts1;
  Edrak::Points3D q2 = pts2;
  RemoveCOMPoints3D(q1, p1);
  RemoveCOMPoints3D(q2, p2);
  // std::ofstream file1("q1.txt");
  // std::ofstream file2("q2.txt");
  // file1 << q1;
  // file2 << q2;

  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < q1.rows(); i++) {
    W += q1.row(i).transpose() * q2.row(i);
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R = U * V.transpose();

  if (R.determinant() < 0) {
    R = -R;
  }

  Eigen::Vector3d t = p1 - R * p2;

  return Edrak::SE3D(Edrak::SO3D(R), t);
}
} // namespace Edrak
