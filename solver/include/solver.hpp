#ifndef SOLVER_HPP_18_10_28_09_56_06
#define SOLVER_HPP_18_10_28_09_56_06 
#include <cassert>
#include <iostream>
#include <limits>
#include "chain.hpp"
#include "quaternion.hpp"

namespace ik {
	namespace FABRIK {

		// prevBoneDir from j-1 towards j, boneDir from j towards j+1
		// t is new position of j+1
		Vector constrainedJointRotation(const Joint* j, Vector boneDir, Vector prevBoneDir, Vector t)
		{
			return boneDir;
			// find projection (O) of new target position (t) onto jointPos.+lDir line
			Vector lDir = prevBoneDir;
			lDir.normalize();
			Vector v = t-j->position;
			Vector o = j->position + lDir*(v).dot(lDir);
			// find distance (S) between t and O
			float s = (t-o).length();
			// rotate and translate (R) t (T) so that O is at 0 and oriented according to x,y axes
			// prevBoneDir = z+ (screen to chair), 
			// j.orientation = y+ (up)
			//Vector Rtr = o*-1;
			//Vector T = t+Rtr;
			//boneDir.normalize();
			//std::cout << boneDir.dot(j->orientation) << std::endl;
			//Quaternion Rrot(prevBoneDir, Vector(0,0,1));
			//Rrot = Rrot * Quaternion(j->orientation, Vector(0,1,0));
			//Rrot.rotateVector(T);

			// find quadrant which T belongs to
			// find conic section which describes the allowed rotation in that quadrant
			// if T is not within the conic section, move it to the closest point on the conic section
			// apply inverse R on T
			return boneDir;
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
			// adjust the position of the "end-effector"
			Vector prevJointPrevPos = j->position;
			Vector prevJointCurPos = (j->position = newPos);

			// normalize the new orientation
			assert(newOrientation.perpendicularize(chain.getJoint(endEffectorID+inc).position-newPos));
			
			// adjust the orientation of the "end-effector"
			Vector prevJointOrientation = (j->orientation = newOrientation);

			// calculate the rotation of the previous bone, if there is one
			Vector prevJointRotation;
			unsigned pjID = endEffectorID-inc;
			bool hasPreceedingBone = false;
			if(pjID < chain.jointCount()) {
				prevJointRotation = (chain.getJoint(endEffectorID).position - chain.getJoint(pjID).position).normalize();
				hasPreceedingBone = true;
			}
			for(int i = endEffectorID+inc; i != int(baseID+inc); i += inc) {
				j = &chain.getJoint(i);
				float boneLength = (j->position-prevJointPrevPos).length();
				Vector jointRotation = (j->position - prevJointCurPos).normalize();
				// update joint orientation (apply the same transform as on the rotation)
				Vector jointPrevRotation = (j->position - prevJointPrevPos).normalize();
				Quaternion r(jointPrevRotation, jointRotation);
				r.rotateVector(j->orientation);
				j->orientation.perpendicularize(jointRotation);
				// constraints can be applied only if there is a preceeding bone - its rotation vector is needed
				if(hasPreceedingBone) {
					// apply orientation constraints
					j->orientation = constrainedJointOrientation(j, prevJointOrientation, jointRotation, prevJointRotation);

					// apply rotation constraints
					//jointRotation = constrainedJointRotation(j, jointRotation, prevJointRotation, newPos + jointRotation*(j->position-chain.getJoint(i+inc).position).length());
					// normalize the orientation (the boneDir might have changed)
					//j->orientation.perpendicularize(jointRotation);
				}

				// update the position of the current joint
				prevJointPrevPos = j->position;
				prevJointCurPos = (j->position = prevJointCurPos + jointRotation*boneLength);
				prevJointRotation = jointRotation;
				prevJointOrientation = j->orientation;
				hasPreceedingBone = true;
			}
		}

		void solveChain(Chain& chain, unsigned jointID, Vector newPos, Vector newOrientation, float epsilon = 0.1, unsigned maxIter = 10)
		{
			unsigned baseID = chain.baseJointID();
			Vector initialBasePosition = chain.getJoint(baseID).position;
			Vector initialBaseOrientation = chain.getJoint(baseID).orientation;
			unsigned i = 0;
			float lastEpsilon = std::numeric_limits<float>::max();
			do {
				solveChainBidirectional(chain, jointID, baseID, newPos, newOrientation);
				solveChainBidirectional(chain, baseID, jointID, initialBasePosition, initialBaseOrientation);
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
