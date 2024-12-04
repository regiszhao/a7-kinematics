#define _USE_MATH_DEFINES
#include <cmath>
#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
    // extract euler angles
    double x1 = xzx(0) * M_PI / 180;
    double z = xzx(1) * M_PI / 180;
    double x2 = xzx(2) * M_PI / 180;
   
    // rotation matrices
    Eigen::AngleAxisd X1(x1, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Z(z, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd X2(x2, Eigen::Vector3d::UnitX());

    return (Eigen::Affine3d) X2 * Z * X1;
  /////////////////////////////////////////////////////////////////////////////
}
