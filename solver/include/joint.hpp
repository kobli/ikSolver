#ifndef JOINT_HPP_18_10_28_10_08_44
#define JOINT_HPP_18_10_28_10_08_44 
#include <cmath>

namespace ik {
	template<typename V>
	struct Joint {
		V position;
		float maxXAngleCW = M_PI;
		float maxXAngleCCW = M_PI;
	};
}
#endif /* JOINT_HPP_18_10_28_10_08_44 */
