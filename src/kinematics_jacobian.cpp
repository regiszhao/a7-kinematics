#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
	J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);

	double h = 0.0000001;
	Eigen::VectorXd tips = transformed_tips(skeleton, b);

	// loop through bones
	for (int i = 0; i < skeleton.size(); i++) {
		// each euler angle
		for (int j = 0; j < 3; j++) {
			Skeleton skeleton_h = skeleton; // reset skeleton
			skeleton_h[i].xzx(j) += h;
			Eigen::VectorXd tips_h = transformed_tips(skeleton_h, b);
			Eigen::VectorXd dx = tips_h - tips;
			// update entire column
			J.col(i * 3 + j) = dx / h;
		}
	}
  /////////////////////////////////////////////////////////////////////////////
}
