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
		float maxRotAngleX;
		float maxRotAngleY;
		float maxRotAngleNX;
		float maxRotAngleNY;
	};
}
#endif /* JOINT_HPP_18_10_28_10_08_44 */
