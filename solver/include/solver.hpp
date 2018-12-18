#ifndef SOLVER_HPP_18_10_28_09_56_06
#define SOLVER_HPP_18_10_28_09_56_06 
#include <cassert>
#include <iostream>
#include <limits>
#include "chain.hpp"

namespace ik {
	namespace FABRIK {

		Vector constrainedJointRotation(const Joint* j, Vector boneDir, Vector prevBoneDir)
		{
			return boneDir;
			/*
			if(prevBoneDir.length() != 0 && j != nullptr) {
				using namespace std;

				// rotate the bone vector into "ground-plane of prevBoneDir" (xz)
				float prevBoneDirZAngleDeg = atan2(prevBoneDir.y, prevBoneDir.x)/M_PI*180; // the angle is measured CCW
				boneDir.rotateXYBy(-prevBoneDirZAngleDeg); // rotates the vector counter-clockwise

				float boneDirZAngle = atan2(boneDir.y, boneDir.x);
				boneDirZAngle = min(boneDirZAngle, j->maxZAngleCCW);
				boneDirZAngle = max(boneDirZAngle, -j->maxZAngleCW);

				Vector r(0);
				r.x = 1;
				r.rotateXYBy(boneDirZAngle/M_PI*180);
				r.rotateXYBy(prevBoneDirZAngleDeg);
				r.normalize();
				return r;
			}
			else
				return boneDir;
				*/
		}


		void solveChainBidirectional(Chain& chain, unsigned endEffectorID, unsigned baseID, Vector newPos)
		{
			assert(endEffectorID != baseID);
			int inc = endEffectorID > baseID? -1: 1;

			Joint* j = &chain.getJoint(endEffectorID);
			Vector prevJointPrevPos = j->position;
			Vector prevJointCurPos = (j->position = newPos);
			Vector prevJointRotation = (chain.getJoint(endEffectorID+inc).position - newPos).normalize();
			Vector defV(0);
			defV.x = 1;
			prevJointRotation = constrainedJointRotation(j, prevJointRotation, defV);

			for(int i = endEffectorID+inc; i != int(baseID+inc); i += inc) {
				j = &chain.getJoint(i);
				float boneLength = (j->position-prevJointPrevPos).length();
				newPos = prevJointCurPos + prevJointRotation*boneLength;
				if(i != int(baseID)) {
					Vector jointRotation = (chain.getJoint(i+inc).position-newPos).normalize();
					prevJointRotation = constrainedJointRotation(j, jointRotation, prevJointRotation);
				}
				prevJointCurPos = newPos;
				prevJointPrevPos = j->position;
				j->position = newPos;
			}
		}

		void solveChain(Chain& chain, unsigned jointID, Vector newPos, float epsilon = 0.1, unsigned maxIter = 10)
		{
			unsigned baseID = chain.baseJointID();
			Vector initialBasePosition = chain.getJoint(baseID).position;
			unsigned i = 0;
			float lastEpsilon = std::numeric_limits<float>::max();
			do {
				solveChainBidirectional(chain, jointID, baseID, newPos);
				solveChainBidirectional(chain, baseID, jointID, initialBasePosition);
				float newEpsilon = (newPos-chain.getJoint(jointID).position).length();
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
