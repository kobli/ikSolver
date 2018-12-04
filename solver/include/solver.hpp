#ifndef SOLVER_HPP_18_10_28_09_56_06
#define SOLVER_HPP_18_10_28_09_56_06 
#include <cassert>
#include <iostream>
#include <limits>
#include "chain.hpp"

namespace ik {
	namespace FABRIK {

		template<typename V>
			V constrainedJointRotation(const Joint<V>* j, V boneDir, V prevBoneDir)
			{
				if(prevBoneDir.getLength() != 0 && j != nullptr) {
					using namespace std;
					float prevBoneDirAngleDeg = atan2(prevBoneDir.Y, prevBoneDir.X)/M_PI*180; // the angle is measured CCW
					boneDir.rotateXYBy(-prevBoneDirAngleDeg); // rotates the vector counter-clockwise

					float boneDirAngle = atan2(boneDir.Y, boneDir.X);
					boneDirAngle = min(boneDirAngle, j->maxXAngleCCW);
					boneDirAngle = max(boneDirAngle, -j->maxXAngleCW);

					V r(0);
					r.X = 1;
					r.rotateXYBy(boneDirAngle/M_PI*180);
					r.rotateXYBy(prevBoneDirAngleDeg);
					r.normalize();
					return r;
				}
				else
					return boneDir;
			}


		template<typename V>
			void solveChainBidirectional(Chain<V>& chain, unsigned endEffectorID, unsigned baseID, V newPos)
			{
				assert(endEffectorID != baseID);
				int inc = endEffectorID > baseID? -1: 1;

				Joint<V>* j = &chain.getJoint(endEffectorID);
				V prevJointPrevPos = j->position;
				V prevJointCurPos = (j->position = newPos);
				V prevJointRotation = (chain.getJoint(endEffectorID+inc).position - newPos).normalize();
				V defV(0);
				defV.X = 1;
				prevJointRotation = constrainedJointRotation(j, prevJointRotation, defV);

				for(int i = endEffectorID+inc; i != int(baseID+inc); i += inc) {
					j = &chain.getJoint(i);
					float boneLength = (j->position-prevJointPrevPos).getLength();
					newPos = prevJointCurPos + prevJointRotation*boneLength;
					if(i != int(baseID)) {
						V jointRotation = (chain.getJoint(i+inc).position-newPos).normalize();
						prevJointRotation = constrainedJointRotation(j, jointRotation, prevJointRotation);
					}
					prevJointCurPos = newPos;
					prevJointPrevPos = j->position;
					j->position = newPos;
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
