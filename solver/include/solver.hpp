#ifndef SOLVER_HPP_18_10_28_09_56_06
#define SOLVER_HPP_18_10_28_09_56_06 
#include <cassert>
#include <iostream>
#include <limits>
#include "chain.hpp"

namespace ik {
	namespace FABRIK {
		template<typename V>
			void solveChainBidirectional(Chain<V>& chain, unsigned endEffectorID, unsigned baseID, V newPos)
			{
				V prevJointPrevPos = chain.getJoint(endEffectorID).position;
				V prevJointCurPos = newPos;
				assert(endEffectorID != baseID);

				int inc = endEffectorID > baseID? -1: 1;
				for(int i = endEffectorID; i != int(baseID+inc); i += inc) {
					Joint<V>& j = chain.getJoint(i);
					float boneLength = (j.position-prevJointPrevPos).getLength();
					newPos = prevJointCurPos + (j.position-prevJointCurPos).normalize()*boneLength;
					prevJointCurPos = newPos;
					prevJointPrevPos = j.position;
					j.position = newPos;
				}
			}

		template<typename V>
			void solveChain(Chain<V>& chain, unsigned jointID, V newPos, float epsilon = 0.1, unsigned maxIter = 10)
			{
				unsigned baseID = chain.baseJointID();
				V initialBasePosition = chain.getJoint(baseID).position;
				unsigned i = 0;
				float lastEpsilon = std::numeric_limits<float>::max();
				do {
					solveChainBidirectional(chain, jointID, baseID, newPos);
					solveChainBidirectional(chain, baseID, jointID, initialBasePosition);
					float newEpsilon = (newPos-chain.getJoint(jointID).position).getLength();
					if(newEpsilon >= lastEpsilon) {
						std::cerr << "FABRIK is diverging.\n";
					}
					lastEpsilon = newEpsilon;
					++i;
				}while(lastEpsilon > epsilon && i < maxIter);
			}
	}
}
#endif /* SOLVER_HPP_18_10_28_09_56_06 */
