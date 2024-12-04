#include "projected_gradient_descent.h"
#include "line_search.h"

void projected_gradient_descent(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const int max_iters,
  Eigen::VectorXd & z)
{
  /////////////////////////////////////////////////////////////////////////////
	double max_step = 10000;
	for (int i = 0; i < max_iters; i++) {
		// compute gradient and step size
		Eigen::VectorXd gradient = grad_f(z);
		double sigma = line_search(f, proj_z, z, gradient, max_step);

		// update z
		z = z - sigma * gradient;
		proj_z(z);
	}
  /////////////////////////////////////////////////////////////////////////////
}
