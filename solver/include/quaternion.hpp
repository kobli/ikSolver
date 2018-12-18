#ifndef QUATERNION_HPP_18_12_18_16_42_50
#define QUATERNION_HPP_18_12_18_16_42_50 
#include <iostream>
#include "vector.hpp"
static const float EPSILON = 1e-6f;

class Quaternion {
	public:
		Quaternion(Vector Axis, float Angle): axis{Axis}, angle{Angle}
		{}

		Quaternion(Vector rotFrom, Vector rotTo) {
			rotFrom.normalize();
			rotTo.normalize();

			float d = rotFrom.dot(rotTo);
			if(d > 1-EPSILON) { // identical vectors
				angle = 0;
				axis = {1,0,0};
			}
			else if(d < -1 + EPSILON) { // opposite vectors
				angle = M_PI;
				axis = Vector(1,0,0).cross(rotFrom);
				if(axis.length() < EPSILON)
					axis = Vector(0,1,0).cross(rotFrom);
				axis.normalize();
			}
			else {
				axis = rotFrom.cross(rotTo);
				axis.normalize();
				angle = acos(d);
			}
		}

		void rotateVector(Vector& v) const {
			float cosAngle = cos(angle);
			float sinAngle = sin(angle);
			Vector a = axis;
			a.normalize();
			v = v*cosAngle + a.cross(v)*sinAngle + a*(1-cosAngle)*a.dot(v);
		}

		float length() const {
			return sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z + angle*angle);
		}

	private:
		Vector axis;
		float angle;
};
#endif /* QUATERNION_HPP_18_12_18_16_42_50 */
