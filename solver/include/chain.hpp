#ifndef CHAIN_HPP_18_10_28_09_42_25
#define CHAIN_HPP_18_10_28_09_42_25 
#include <vector>
#include "joint.hpp"

/*
 * base joint has fixed position and orientation (typically in model space)
 * orientation vector is the up vector (y+) for the coordinate system of the joint, but it is not always perpendicular to the bone
 * to fully specify the base joint orientation (and to be able to apply the constraints), we also need a front vector (z+),
 * pointing from "previous joint" towards the base joint, which is specified by using one extra joint preceeding the base joint (only its position is relevant)
 *
 * the constraints of the endEffector are ignored since there is no bone going out from it
 *
 * constraints in the base joint define
 * - how much the orientation can differ from the or. of the previous joint
 * - what are the allowed angles of the outgoing bone w.r.t the ingoing bone
 */

namespace ik {
	class Chain {
		public:
			Chain(unsigned jointCount = 0): _baseJointID{1}
			{
				_joints.resize(jointCount);
			}

			// the joints should be added starting from the base
			// expects absolute positions of the joints
			void appendJoint(const Joint& j) {
				_joints.push_back(j);
			}

			Joint& getJoint(unsigned jointID) {
				return _joints[jointID];
			}

			const Joint& getJoint(unsigned jointID) const {
				return _joints[jointID];
			}

			unsigned jointCount() const {
				return _joints.size();
			}

			unsigned baseJointID() const {
				return _baseJointID;
			}

			void setBaseJointID(unsigned bID) {
				_baseJointID = bID;
			}

		private:
			unsigned _baseJointID;
			std::vector<Joint> _joints;
	};

}
#endif /* CHAIN_HPP_18_10_28_09_42_25 */
