#ifndef SOLVER_HPP_18_10_28_09_56_06
#define SOLVER_HPP_18_10_28_09_56_06 
#include <cassert>
#include <iostream>
#include <limits>
#include "chain.hpp"
#include "quaternion.hpp"

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

		Vector constrainedJointOrientation(const Joint* j, Vector prevJointOrientation, Vector boneDir, Vector prevBoneDir)
		{
			// 1) align the boneDir with prevBoneDir
			// 	- apply the same rotation to orientation vector
			// 	- now the two orientation vectors are in a plane perpendicular to the prevBoneDir
			Quaternion q(boneDir, prevBoneDir);
			Vector newOrientation = j->orientation;
			q.rotateVector(newOrientation);
			// 2) determine the angle between Op and O of the previous joint
			Quaternion oo(prevJointOrientation, newOrientation);
			// 3) clamp the angle and apply it
			using namespace std;
			oo.setAngle(min(j->maxCWtwist, max(-j->maxCCWtwist, oo.getAngle())));
			newOrientation = prevJointOrientation;
			oo.rotateVector(newOrientation);
			// 4) apply rotor R to the result (R: -prevBoneDir (+z) => boneDir)
			q.setAngle(-1*q.getAngle());
			q.rotateVector(newOrientation);

			newOrientation.normalize();
			return newOrientation;
		}


		void solveChainBidirectional(Chain& chain, unsigned endEffectorID, unsigned baseID, Vector newPos, Vector newOrientation)
		{
			assert(endEffectorID != baseID);
			int inc = endEffectorID > baseID? -1: 1;

			Joint* j = &chain.getJoint(endEffectorID);
			Vector prevJointPrevPos = j->position;
			Vector prevJointCurPos = (j->position = newPos);
			Vector prevJointRotation = (chain.getJoint(endEffectorID+inc).position - newPos).normalize();
			Vector nullJointRotation(1, 0, 0);
			Vector nullJointOrientation(1, 0, 0);
			j->orientation = newOrientation;
			j->orientation = constrainedJointOrientation(j, nullJointOrientation, prevJointRotation, nullJointRotation);
			Vector prevJointOrientation = j->orientation;
			prevJointRotation = constrainedJointRotation(j, prevJointRotation, nullJointRotation);

			for(int i = endEffectorID+inc; i != int(baseID+inc); i += inc) {
				j = &chain.getJoint(i);
				float boneLength = (j->position-prevJointPrevPos).length();
				newPos = prevJointCurPos + prevJointRotation*boneLength;
				if(i != int(baseID)) {
					Vector jointPrevRotation = chain.getJoint(i+inc).position-chain.getJoint(i).position;
					Vector jointRotation = (chain.getJoint(i+inc).position-newPos).normalize();
					// update the orientation (apply the same transform as on the rotation)
					Quaternion r(jointPrevRotation, jointRotation);
					r.rotateVector(j->orientation);
					j->orientation = constrainedJointOrientation(j, prevJointOrientation, jointRotation, prevJointRotation);
					prevJointOrientation = j->orientation;
					prevJointRotation = constrainedJointRotation(j, jointRotation, prevJointRotation);
				}
				prevJointCurPos = newPos;
				prevJointPrevPos = j->position;
				j->position = newPos;
			}
		}

		void solveChain(Chain& chain, unsigned jointID, Vector newPos, Vector newOrientation, float epsilon = 0.1, unsigned maxIter = 10)
		{
			unsigned baseID = chain.baseJointID();
			Vector initialBasePosition = chain.getJoint(baseID).position;
			unsigned i = 0;
			float lastEpsilon = std::numeric_limits<float>::max();
			do {
				solveChainBidirectional(chain, jointID, baseID, newPos, newOrientation);
				solveChainBidirectional(chain, baseID, jointID, initialBasePosition, chain.getJoint(baseID).orientation);
				float newEpsilon = (newPos-chain.getJoint(jointID).position).length();
				if(newEpsilon >= lastEpsilon) {
					;//std::cerr << "FABRIK is diverging.\n";
				}
				lastEpsilon = newEpsilon;
				++i;
			}while(lastEpsilon > epsilon && i < maxIter);
		}
	}
}
#endif /* SOLVER_HPP_18_10_28_09_56_06 */
