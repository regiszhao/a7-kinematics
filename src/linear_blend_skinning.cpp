#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
	U.resize(V.rows(), 3);

	// initialize temp variable for each deformation
	Eigen::Vector4d v;

	// loop through V
	for (int i = 0; i < V.rows(); i++) {
		v = Eigen::Vector4d::Zero();
		// loop through bones
		for (int j = 0; j < skeleton.size(); j++) {
			if (skeleton[j].weight_index != -1) {
				Eigen::Vector4d v_rest = Eigen::Vector4d(V(i, 0), V(i, 1), V(i, 2), 1);
				v += W(i, skeleton[j].weight_index) * (T[j] * v_rest);
			}
		}

		// update U
		U(i, 0) = v(0);
		U(i, 1) = v(1);
		U(i, 2) = v(2);
	}

  /////////////////////////////////////////////////////////////////////////////
}
