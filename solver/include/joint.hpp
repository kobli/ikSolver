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
		//TODO how to define the allowed rotation ranges? (4 angles .. names?)
		//float maxZAngleCW = M_PI;
		//float maxZAngleCCW = M_PI;
	};
}
#endif /* JOINT_HPP_18_10_28_10_08_44 */
