#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
	// if no keyframes, return 0
	if (keyframes.size() == 0) {
		return Eigen::Vector3d::Zero();
	}
	// if only one keyframe, return that keyframe
	else if (keyframes.size() == 1) {
		return keyframes[0].second;
	}
	// 2 or more key frames:
	else {

		// periodic
		t = fmod(t, keyframes.back().first);

		// find index interval that contains t
		int i = 0;
		while ((i < keyframes.size() - 1) && (keyframes[i].first < t)) {
			i++;
		}

		// if matches first or last keyframe, return that keyframe
		if (i == 0 || i == keyframes.size() - 1) {
			return keyframes[i].second;
		}

		// get times of 4 keyframes
		double t0 = (i > 0) ? keyframes[i - 1].first : keyframes[i].first;
		double t1 = keyframes[i].first;
		double t2 = keyframes[i + 1].first;
		double t3 = (i + 2 < keyframes.size()) ? keyframes[i + 2].first : keyframes[i + 1].first;

		// corresponding keyframes
		Eigen::Vector3d p0 = (i > 0) ? keyframes[i - 1].second : keyframes[i].second;
		Eigen::Vector3d p1 = keyframes[i].second;
		Eigen::Vector3d p2 = keyframes[i + 1].second;
		Eigen::Vector3d p3 = (i + 2 < keyframes.size()) ? keyframes[i + 2].second : keyframes[i + 1].second;

		// calculate tangents
		Eigen::Vector3d m0 = (p2 - p0) / (t2 - t0);
		Eigen::Vector3d m1 = (p3 - p1) / (t3 - t1);

		// arbitrary interval substitute
		double t = (t - t1) / (t2 - t1);

		double tp2 = pow(t, 2);
		double tp3 = pow(t, 3);
		Eigen::Vector3d res = (2 * tp3 - 3 * tp2 + 1) * p1 + (tp3 - 2 * tp2 + t) * m0 + (-2 * tp3 + 3 * tp2) * p2 + (tp3 - tp2) * m1;

		return res;
	}
  /////////////////////////////////////////////////////////////////////////////
}
