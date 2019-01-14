#ifndef CHAIN_HPP_18_10_28_09_42_25
#define CHAIN_HPP_18_10_28_09_42_25 
#include <vector>
#include "joint.hpp"

/*
 * The joint positions and directions are absolute.
 * Base joint has fixed position and orientation. To be able to enforce rotation constraints for the base joint, it requires one more joint preceeding it (with arbitrary position). Therefore baseJointID should typically be 1.
 * Orientation vector is the up vector (y+) for the coordinate system of the joint, but it is not always kept perpendicular to the bone.
 * The constraints of the endEffector are irrelevant since there is no bone going out from it.
 *
 * constraints in a joint define
 * - how much the orientation can differ from the or. of the previous joint
 * - what are the allowed angles of the outgoing bone w.r.t the ingoing bone
 *
 * The baseJointID can be adjusted at any time to fixate the part of the chain preceeding the given joint.
 */

namespace ik {
	class Chain {
		public:
			Chain(unsigned jointCount = 0);

			// the joints should be added starting from the base
			// expects absolute positions of the joints
			void appendJoint(const Joint& j);
			Joint& getJoint(unsigned jointID);
			const Joint& getJoint(unsigned jointID) const;
			unsigned jointCount() const;
			unsigned baseJointID() const;
			void setBaseJointID(unsigned bID);

		private:
			unsigned _baseJointID;
			std::vector<Joint> _joints;
	};

}
#endif /* CHAIN_HPP_18_10_28_09_42_25 */
