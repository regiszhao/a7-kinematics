#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
	// initialize sigma to max_step
	double sigma = max_step;

	Eigen::VectorXd z_new = z - sigma * dz;
	proj_z(z_new);

	// while energy of new z is still greater than original
	while (f(z_new) > f(z)) {
		// update sigma
		sigma = 0.5 * sigma;
		
		// update new z
		z_new = z - sigma * dz;
		proj_z(z_new);
	}

	return sigma;
  /////////////////////////////////////////////////////////////////////////////
}
