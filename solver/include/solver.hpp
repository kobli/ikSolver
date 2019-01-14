#ifndef SOLVER_HPP_18_10_28_09_56_06
#define SOLVER_HPP_18_10_28_09_56_06 
#include <cassert>
#include <iostream>
#include <limits>
#include <functional>
#include "chain.hpp"
#include "quaternion.hpp"

namespace ik {
	namespace FABRIK {

		// prevBoneDir from j-1 towards j, t is the new position of j+1
		// returns adjusted target position
		Vector constrainedJointRotation(const Joint* j, Vector t, Vector prevBoneDir);

		Vector constrainedJointOrientation(const Joint* j, Vector prevJointOrientation, Vector boneDir, Vector prevBoneDir);

		// The jointID should be the ID of the endEffector. It does not necessarily have to be the end of the chain, 
		// but in that case the rotation constraints in that joint will not be enforced.
		// To enforce them, you should fixate the previous part of the chain (set baseID to jointID) and rerun
		// solveChain with jointID=chain.jointCount()-1
		void solveChain(Chain& chain, unsigned jointID, Vector newPos, Vector newOrientation, float epsilon = 0.1, unsigned maxIter = 10);
	}
}
#endif /* SOLVER_HPP_18_10_28_09_56_06 */
