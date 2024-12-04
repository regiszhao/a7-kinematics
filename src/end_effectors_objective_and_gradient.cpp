#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////


    // computes least squares objective value given #bones list of euler angles
    f = [&](const Eigen::VectorXd & A)->double
        {
            Skeleton skeleton_copy = copy_skeleton_at(skeleton, A);
            // get positions of tips
            Eigen::VectorXd tips = transformed_tips(skeleton_copy, b);

            // we are calculating sum of squared norms -> same as dot product
            return (tips - xb0).dot(tips - xb0);
        };



    // computes least squares objective gradient
    grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
        {
            // tips
            Skeleton skeleton_copy = copy_skeleton_at(skeleton, A);
            Eigen::VectorXd tips = transformed_tips(skeleton_copy, b);
            // jacobian
            Eigen::MatrixXd J;
            kinematics_jacobian(skeleton_copy, b, J);
			// https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
			// gradient = 2 * J^T * dE/dx
            Eigen::VectorXd dE_dx = tips - xb0;
            return 2 * J.transpose() * dE_dx;
        };




    // projects euler angles onto skeleton's joint angles
    proj_z = [&](Eigen::VectorXd & A)
        {
            assert(skeleton.size()*3 == A.size());
            // loop through bones
            for (int i = 0; i < skeleton.size(); i++) {
                // for each euler angle
                for (int j = 0; j < 3; j++) {
                    // clamp to bounds
                    A[i * 3 + j] = std::max(skeleton[i].xzx_min(j), std::min(skeleton[i].xzx_max(j), A[i * 3 + j]));
                }
            }
        };
  /////////////////////////////////////////////////////////////////////////////
}
