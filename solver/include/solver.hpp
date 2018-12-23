#ifndef SOLVER_HPP_18_10_28_09_56_06
#define SOLVER_HPP_18_10_28_09_56_06 
#include <cassert>
#include <iostream>
#include <limits>
#include <functional>
#include "chain.hpp"
#include "quaternion.hpp"

namespace ik {
	namespace FABRIK {
		void getConstraintAnglesBasedOnQuadrant(const Vector& v, float thX, float thNX, float thY, float thNY, float& thx, float& thy)
		{
			// the choice of the second angle on a quadrant border does not matter
			if(v.x >= 0) {
				if(v.y >= 0) { // quadrant I
					thx = thX;
					thy = thY;
				}
				else { // quadrant IV
					thx = thX;
					thy = thNY;
				}
			}
			else {
				if(v.y >= 0) { // quadrant II
					thx = thNX;
					thy = thY;
				}
				else { // quadrant III
					thx = thNX;
					thy = thNY;
				}
			}
		}

		Vector findClosestPointOnFunction(Vector t, std::function<float(float)> f, float x1, float x2, float yEps = 0.1)
		{
			t.z = 0;
			float y1 = f(x1);
			float y2 = f(x2);
			auto d = [&](const Vector& v) {
				return (v-t).length();
			};
			float yDelta = 2*yEps; // anything that is larger than yEps will do
			float prevDelta = 2*yDelta;
			while(yDelta > yEps && x1 != x2 && prevDelta != yDelta) {
				float xMid = (x1+x2)/2;
				float yMid = f(xMid);
				float dm = d({xMid,yMid,0});
				float d1 = d({x1,y1,0});
				float d2 = d({x2,y2,0});
				prevDelta = yDelta;
				if(dm < d1) {
					x1 = xMid;
					yDelta = fabs(y1-yMid);
					y1 = yMid;
				}
				else if(dm < d2) {
					x2 = xMid;
					yDelta = fabs(y2-yMid);
					y2 = yMid;
				}
			}
			return {x1, y1, 0};
		}

		// currently works only when all thetas are smaller than PI/2
		Vector nearestPointOnConicSectionIfOutside(const Vector& t, float thX, float thNX, float thY, float thNY, float s)
		{
			// find the closest point to t (nt) lying on the conicsection
			Vector nt;
			if(thX == thNX && thX == thY && thX == thNY) { // circle
				Vector tDir = t;
				tDir.z = 0;
				tDir.normalize();
				float r = s*tan(thX);
				nt = tDir*r;
			}
			else if(
					(thX  < M_PI/2) == 
					(thNX < M_PI/2) == 
					(thY  < M_PI/2) == 
					(thNY < M_PI/2)) { // ellipsoid
				float thx,
							thy;
				getConstraintAnglesBasedOnQuadrant(t, thX, thNX, thY, thNY, thx, thy);
				// a is the half-axis matching with x, it is not necessarilly the major axis
				float a = s*tan(thx);
				float b = s*tan(thy);
				float sign = (t.y>=0)?1:-1;
				auto ellipse = [&](float x) {
					return sign*b/a*sqrt(a*a - x*x);
				};
				float x1,
							x2;
				if(t.x < 0) {
					x1 = -a;
					x2 = 0;
				}
				else {
					x1 = 0;
					x2 = a;
				}
				nt = findClosestPointOnFunction(t, ellipse, x1, x2);
			}
			else { // parabolic shape
				//TODO
				assert(false);
			}

			// if t is not within the conic section, move it to the closest point on the conic section (nt)
			Vector tt = t;
			tt.z = 0;
			if(tt.length() > nt.length()) {
				nt.z = t.z;
				return nt;
			}
			else
				return t;
		}

		// prevBoneDir from j-1 towards j, boneDir from j towards j+1
		Vector constrainedJointRotation(const Joint* j, Vector boneDir, Vector prevBoneDir)
		{
			boneDir.normalize();
			prevBoneDir.normalize();
			Vector t = j->position + boneDir;
			// find projection (O) of new target position (t) onto jointPos.+lDir line
			Vector lDir = prevBoneDir;
			lDir.normalize();
			Vector v = t-j->position;
			Vector o = j->position + lDir*(v).dot(lDir);
			// find distance (S) between t and O
			float s = (t-o).length();
			// rotate and translate (using transform R) t (T) so that O is at 0 and oriented according to x,y axes
			// prevBoneDir = z+ (screen to chair), 
			// j.orientation = y+ (up)
			Vector Rtr = o*-1;
			Vector T = t+Rtr;
			Quaternion Rrot1(prevBoneDir, Vector(0,0,1));
			Rrot1.rotateVector(T);
			Quaternion Rrot2(j->orientation, Vector(0,1,0));
			Rrot2.rotateVector(T);

			// find the nearest point on conic section if T lies outside of it
			T = nearestPointOnConicSectionIfOutside(T, j->maxRotAngleX, j->maxRotAngleNX, j->maxRotAngleY, j->maxRotAngleNY, s);

			// apply inverse R on T
			Rrot2.setAngle(-Rrot2.getAngle());
			Rrot1.setAngle(-Rrot1.getAngle());
			Rrot2.rotateVector(T);
			Rrot1.rotateVector(T);
			T = T-Rtr;

			T.normalize();
			return T;
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


		void solveChainBidirectional(Chain& chain, unsigned endEffectorID, unsigned baseID, Vector endEffectorNewPos, Vector newOrientation)
		{
			assert(endEffectorID != baseID);
			int inc = endEffectorID > baseID? -1: 1;

			Joint* j = &chain.getJoint(endEffectorID);
			// adjust the position of the "end-effector"
			Vector prevJointPrevPos = j->position;
			Vector prevJointCurPos = (j->position = endEffectorNewPos);

			// normalize the new orientation
			assert(newOrientation.perpendicularize(chain.getJoint(endEffectorID+inc).position-endEffectorNewPos));
			
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
					jointRotation = constrainedJointRotation(j, jointRotation, prevJointRotation);
					// normalize the orientation (the boneDir might have changed)
					j->orientation.perpendicularize(jointRotation);
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
