#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function


// recursive function to calculate transformation matrix
void recurse_T(
	const Skeleton& skeleton,
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& T,
	int idx,
	std::vector<bool>& visited)
{
	int parent_idx = skeleton[idx].parent_index;
	// base cases: parent == -1 or already visited
	// if root set to identity
	if (parent_idx == -1) {
		T[idx] = Eigen::Affine3d::Identity();
		// update visited
		visited[idx] = true;
	}
	// else, set to T_parent * rest_T * rotation matrix * rest_T_inverse
	else {
		if (visited[parent_idx] == false) { // if parent isn't set yet, recurse
			recurse_T(skeleton, T, parent_idx, visited);
		}
		T[idx] = Eigen::Affine3d(T[parent_idx] * skeleton[idx].rest_T * euler_angles_to_transform(skeleton[idx].xzx) * skeleton[idx].rest_T.inverse());
		visited[idx] = true;
	}
}





void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
	T.resize(skeleton.size(),Eigen::Affine3d::Identity());

	// initialize visited list
	std::vector<bool> visited(skeleton.size(), false);

	// loop through bones
	for (int i = 0; i < skeleton.size(); i++) {
		// if index hasn't been visited, run recursive function
		if (visited[i] == false) {
			recurse_T(skeleton, T, i, visited);
		}
	}
  /////////////////////////////////////////////////////////////////////////////
}
