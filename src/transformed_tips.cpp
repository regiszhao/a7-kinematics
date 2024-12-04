#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
	// forward kinematics
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > T;
	forward_kinematics(skeleton, T);

	Eigen::VectorXd tips = Eigen::VectorXd::Zero(3 * b.size());
	Eigen::Vector4d tip;
	Eigen::Vector4d canon_tip;

	// loop through b
	for (int i = 0; i < b.size(); i++) {
		// transform * rest_T * canonical tip position
		canon_tip = Eigen::Vector4d(skeleton[b[i]].length, 0, 0, 1);
		tip = T[b(i)] * skeleton[b[i]].rest_T * canon_tip;

		// update b
		tips[i * 3] = tip(0);
		tips[i * 3 + 1] = tip(1);
		tips[i * 3 + 2] = tip(2);
	}

	return tips;

  /////////////////////////////////////////////////////////////////////////////
}
