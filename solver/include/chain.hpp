#ifndef CHAIN_HPP_18_10_28_09_42_25
#define CHAIN_HPP_18_10_28_09_42_25 
#include <vector>
#include "joint.hpp"

namespace ik {
	template<typename V>
	class Chain {
		public:
			Chain(): _baseJointID{0}
			{}

			// the joints should be added starting from the base
			// expects absolute positions of the joints
			void appendJoint(const Joint<V>& j) {
				_joints.push_back(j);
			}

			Joint<V>& getJoint(unsigned jointID) {
				return _joints[jointID];
			}

			const Joint<V>& getJoint(unsigned jointID) const {
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
			std::vector<Joint<V>> _joints;
	};

}
#endif /* CHAIN_HPP_18_10_28_09_42_25 */
