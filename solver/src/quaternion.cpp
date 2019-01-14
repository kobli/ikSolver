#include "quaternion.hpp"

static const float EPSILON = 1e-6f;

Quaternion::Quaternion(Vector Axis, float Angle): axis{Axis}, angle{Angle}
{}

Quaternion::Quaternion(Vector rotFrom, Vector rotTo) {
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

void Quaternion::rotateVector(Vector& v) const {
	float cosAngle = cos(angle);
	float sinAngle = sin(angle);
	Vector a = axis;
	a.normalize();
	v = v*cosAngle + a.cross(v)*sinAngle + a*(1-cosAngle)*a.dot(v);
}

float Quaternion::length() const {
	return sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z + angle*angle);
}

float Quaternion::getAngle() const {
	return angle;
}

void Quaternion::setAngle(float a) {
	angle = a;
}
