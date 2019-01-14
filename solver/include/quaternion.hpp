#ifndef QUATERNION_HPP_18_12_18_16_42_50
#define QUATERNION_HPP_18_12_18_16_42_50 
#include <iostream>
#include "vector.hpp"

class Quaternion {
	public:
		Quaternion(Vector Axis, float Angle);
		Quaternion(Vector rotFrom, Vector rotTo);
		void rotateVector(Vector& v) const;
		float length() const;
		float getAngle() const;
		void setAngle(float a);

	private:
		Vector axis;
		float angle;
};
#endif /* QUATERNION_HPP_18_12_18_16_42_50 */
