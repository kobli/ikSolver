#include "chain.hpp"

namespace ik {
	Chain::Chain(unsigned jointCount): _baseJointID{1}
	{
		_joints.resize(jointCount);
	}

	// the joints should be added starting from the base
	// expects absolute positions of the joints
	void Chain::appendJoint(const Joint& j) {
		_joints.push_back(j);
	}

	Joint& Chain::getJoint(unsigned jointID) {
		return _joints[jointID];
	}

	const Joint& Chain::getJoint(unsigned jointID) const {
		return _joints[jointID];
	}

	unsigned Chain::jointCount() const {
		return _joints.size();
	}

	unsigned Chain::baseJointID() const {
		return _baseJointID;
	}

	void Chain::setBaseJointID(unsigned bID) {
		_baseJointID = bID;
	}
}
