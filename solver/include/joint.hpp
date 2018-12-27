#ifndef JOINT_HPP_18_10_28_10_08_44
#define JOINT_HPP_18_10_28_10_08_44 
#include <cmath>
#include "vector.hpp"

namespace ik {
	struct Joint {
		Vector position;
		Vector orientation;
		float maxCWtwist;
		float maxCCWtwist;
		float maxRotAngleX  = M_PI/2-0.001;
		float maxRotAngleY  = M_PI/2-0.001;
		float maxRotAngleNX = M_PI/2-0.001;
		float maxRotAngleNY = M_PI/2-0.001;
	};
}
#endif /* JOINT_HPP_18_10_28_10_08_44 */
