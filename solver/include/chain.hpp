#ifndef CHAIN_HPP_18_10_28_09_42_25
#define CHAIN_HPP_18_10_28_09_42_25 
#include <vector>
#include "joint.hpp"

namespace ik {
	class Chain {
		public:
			Chain(): _baseJointID{0}
			{}

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

		private:
			unsigned _baseJointID;
			std::vector<Joint> _joints;
	};

}
#endif /* CHAIN_HPP_18_10_28_09_42_25 */
