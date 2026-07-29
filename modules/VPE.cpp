module;

#include <algorithm>
#include <cmath>

module VEPhysicsEngine;

namespace geometry {

	//--------------------------------Begin-Cloth-Simulation-Stuff----------------------------------
	// by Felix Neumann

	/// <summary>
	/// Alpha Max Plus Beta Min - Approximates square root of the sum of two squares (magnitude of a
	/// 2d vector).
	/// https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
	/// </summary>
	real alphaMaxPlusBetaMin(real a, real b)
	{
		real absA = fabs(a);
		real absB = fabs(b);
		if (absA > absB)
			return (real)(0.96043387010342 * absA + 0.397824734759316 * absB);

		return (real)(0.96043387010342 * absB + 0.397824734759316 * absA);
	}

	/// <summary>
	/// Alpha Max Plus Beta Min extented to 3 dimensions. Approximates the magnitude of a 3d vector.
	/// https://math.stackexchange.com/questions/1282435/
	/// https://stackoverflow.com/questions/1582356/
	/// </summary>
	real alphaMaxPlusBetaMedPlusGammaMin(real a, real b, real c)
	{
		real absA = fabs(a);
		real absB = fabs(b);
		real absC = fabs(c);

		real min = std::min(absA, std::min(absB, absC));
		real max = std::max(absA, std::max(absB, absC));
		real med = std::max(std::min(absA, absB), std::min(std::max(absA, absB), absC));

		return (real)(0.939808635172325 * max + 0.389281482723725 * med + 0.29870618761438 * min);
	}
	//---------------------------------End-Cloth-Simulation-Stuff-----------------------------------

}
