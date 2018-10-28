#ifndef CHAIN_HPP_18_10_28_09_42_25
#define CHAIN_HPP_18_10_28_09_42_25 
#include <vector>
#include "joint.hpp"

namespace ik {
	template<typename V>
	class Chain {
		public:
			// the joints should be added starting from the base
			// expects absolute positions of the joints
			void appendJoint(const Joint<V>& j);

		private:
			unsigned _baseJointID;
			std::vector<Joint<V>> _joints;
	};

}
#endif /* CHAIN_HPP_18_10_28_09_42_25 */
