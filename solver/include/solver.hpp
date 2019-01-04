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

		Vector findClosestPointOnFunction(Vector t, std::function<float(float)> f, float x1, float x2, float xEps = 1)
		{
			t.z = 0;
			float y1 = f(x1);
			float y2 = f(x2);
			auto d = [&](const Vector& v) {
				return (v-t).length();
			};
			float xDelta = 2*xEps; // anything that is larger than xEps will do
			float prevDelta = 2*xDelta;
			while(xDelta > xEps && x1 != x2 && prevDelta != xDelta) {
				float xMid = (x1+x2)/2;
				float yMid = f(xMid);
				float dm = d({xMid,yMid,0});
				float d1 = d({x1,y1,0});
				float d2 = d({x2,y2,0});
				prevDelta = xDelta;
				if(dm < d1) {
					xDelta = fabs(x1-yMid);
					x1 = xMid;
					y1 = yMid;
				}
				else if(dm < d2) {
					xDelta = fabs(x2-xMid);
					x2 = xMid;
					y2 = yMid;
				}
			}
			return {x1, y1, 0};
		}

		// currently works only when all thetas are smaller than PI/2
		Vector nearestPointOnConicSection(const Vector& t, float thx, float thy, float s, float l, bool& tOutside, int targetHemisphere = 1)
		{
			// avoid zero-length semi-axes
			if(thx == 0)
				thx += 0.0001;
			if(thy == 0)
				thy += 0.0001;
			// find the closest point to t (nt) lying on the conicsection
			Vector nt;
			if(thx == thy) { // circle
				Vector tDir = t;
				tDir.z = 0;
				if(tDir.length() == 0)
					nt = t;
				else {
					tDir.normalize();
					float r = s*tan(thx);
					nt = tDir*r;
				}
			}
			else if((thx  < M_PI/2) == (thy < M_PI/2)) { // ellipsoid
				// a is the half-axis matching with x, it is not necessarilly the major axis
				float a = s*tan(thx);
				float b = s*tan(thy);
				float sign = (t.y>=0)?1:-1;
				auto ellipse = [&](float x)->float {
					if(a == 0 || b == 0)
						return 0;
					else
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
				assert(nt.length() == nt.length());
			}
			else { // parabolic shape
				if(thy > M_PI_2) { // opening in y+ / y- direction
					float sign = (t.y>=0)?1:-1;
					Vector v(0, targetHemisphere*sign*s*tan(thy), 0); // vertex
					Vector p(s*tan(thx), 0, 0); // point
					float a = (p.y-v.y)/pow(p.x-v.x,2);
					a = fabs(a);
					auto parabola = [&](float x)->float {
						return sign*a*pow(x-v.x, 2)+v.y;
					};
					tOutside = parabola(t.x)*sign > 0;
					float x1,x2;
					//TODO FIXME? are there any requirements for the closestPoint? maybe use different (tighter) bounds
					if(t.x < 0) {
						x1 = -l;
						x2 = 0;
					}
					else {
						x1 = 0;
						x2 = l;
					}
					nt = findClosestPointOnFunction(t, parabola, x1, x2);
					assert(nt.length() == nt.length());
				}
				else { //TODO opening in x direction
					assert(false);
				}
			}
			nt.z = 0;
			return nt;
		}

		// prevBoneDir from j-1 towards j, t is the new position of j+1
		// returns adjusted target position
		Vector constrainedJointRotation(const Joint* j, Vector t, Vector prevBoneDir)
		{
			prevBoneDir.normalize();
			float boneLength = (j->position-t).length();
			// make sure t does not project onto j->position (which would cause the cut of the cone to have zero area
			// => we would not be able to calculate the new position of t)
			if((t-j->position).normalize().dot(prevBoneDir) == 0)
				t = t + prevBoneDir*0.0001;
			// find projection (O) of new target position (t) onto jointPos.+lDir line
			Vector lDir = prevBoneDir;
			Vector v = t-j->position;
			Vector o = j->position + lDir*(v).dot(lDir);
			// find distance (S) between j and O
			float s = (j->position-o).length();
			bool targetInUpperHemisphere = (o-j->position).dot(prevBoneDir) > 0;
			// rotate and translate (using transform R) t (T) so that O is at 0 and oriented according to x,y axes
			// prevBoneDir = z+ (screen to chair), 
			// j.orientation = y+ (up)
			Vector T = t;
			Vector Rtr = o;
			T = T-Rtr;
			Quaternion Rrot1(prevBoneDir, Vector(0,0,1));
			Rrot1.rotateVector(T);
			Vector orientation = j->orientation;
			orientation.perpendicularize(prevBoneDir);
			Quaternion Rrot2(orientation, Vector(0,1,0));
			Rrot2.rotateVector(T);
			T.z = 0;
			assert(T.length() == T.length());

			// if T lies outside the conic section, replace it with the nearest point on the conic section (nT)
			float thx,
						thy;
			getConstraintAnglesBasedOnQuadrant(T, j->maxRotAngleX, j->maxRotAngleNX, j->maxRotAngleY, j->maxRotAngleNY, thx, thy);
			bool tOutside;
			Vector nT = nearestPointOnConicSection(T, thx, thy, s, boneLength, tOutside);
			assert(nT.length() == nT.length());
			// adjust T if necessary
			if(thx < M_PI_2 && thy < M_PI_2) {
				if(targetInUpperHemisphere) {
					if(T.length() > nT.length())
						T = nT;
				}
				else {
					T = nT;
					// move T along l so it lies on the conic section
					Rtr = Rtr + prevBoneDir*2*s;
				}
			}
			else if(thx > M_PI_2 && thy > M_PI_2) {
				if(targetInUpperHemisphere) {
					// do nothing
				}
				else {
					if(T.length() < nT.length())
						T = nT;
				}
			}
			else {
				if(targetInUpperHemisphere) {
					if(tOutside)
						T = nT;
				}
				else {
					nT = nearestPointOnConicSection(T, 0.001, thy, s, boneLength, tOutside, -1);
					assert(nT.length() == nT.length());
					if(tOutside)
						T = nT;
				}
			}

			// apply inverse R on T
			Rrot2.setAngle(-Rrot2.getAngle());
			Rrot1.setAngle(-Rrot1.getAngle());
			Rrot2.rotateVector(T);
			Rrot1.rotateVector(T);
			T = T+Rtr;

			return T;
		}

		Vector constrainedJointOrientation(const Joint* j, Vector prevJointOrientation, Vector boneDir, Vector prevBoneDir)
		{
			// 1) align the boneDir with prevBoneDir
			// 	- apply the same rotation to orientation vector
			// 	- now the two orientation vectors are in a plane perpendicular to the prevBoneDir
			Quaternion q(boneDir, prevBoneDir);
			Vector newOrientation = j->orientation;
			newOrientation.perpendicularize(prevBoneDir);
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

			const Vector eeInitialPos = chain.getJoint(endEffectorID).position;
			const Vector baseInitialPos = chain.getJoint(baseID).position;

			// adjust the position of the "end-effector"
			Vector prevJointPrevPos = j->position;
			Vector prevJointCurPos = (j->position = endEffectorNewPos);
			
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
				// constraints can be applied only if there is a preceeding bone - its rotation vector is needed
				if(hasPreceedingBone) {
					// apply orientation constraints
					j->orientation = constrainedJointOrientation(j, prevJointOrientation, jointRotation, prevJointRotation);

					// apply rotation constraints
					jointRotation = (constrainedJointRotation(&chain.getJoint(i-inc), prevJointCurPos + jointRotation*boneLength, prevJointRotation)-prevJointCurPos).normalize();
				}

				// update the position of the current joint
				prevJointPrevPos = j->position;
				prevJointCurPos = (j->position = prevJointCurPos + jointRotation*boneLength);
				prevJointRotation = jointRotation;
				prevJointOrientation = j->orientation;
				hasPreceedingBone = true;
			}

			// preserve bone lengths when the base-endEffector subchain does not lie at the tip of the chain
			Vector baseDelta = chain.getJoint(baseID).position - baseInitialPos;
			Vector eeDelta   = chain.getJoint(endEffectorID).position - eeInitialPos;
			Vector delta = baseDelta;
			unsigned i = baseID;
			if(endEffectorID > baseID) {
				i = endEffectorID;
				delta = eeDelta;
			}
			for(i += 1; i < chain.jointCount(); ++i) {
				Vector &p = chain.getJoint(i).position;
				p = p + delta;
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
