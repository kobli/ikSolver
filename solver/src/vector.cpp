#include "vector.hpp"

Vector::Vector(float v): x{v}, y{v}, z{v}
{}

Vector::Vector(float X, float Y, float Z): x{X}, y{Y}, z{Z}
{}

Vector Vector::operator+(const Vector& o) const {
	return Vector(x+o.x, y+o.y, z+o.z);
}

Vector Vector::operator-(const Vector& o) const {
	return Vector(x-o.x, y-o.y, z-o.z);
}

Vector Vector::operator*(float v) const {
	return Vector(x*v, y*v, z*v);
}

Vector& Vector::operator/=(float v) {
	x /= v;
	y /= v;
	z /= v;
	return *this;
}

float Vector::length() const {
	return std::sqrt(x*x + y*y + z*z);
}

Vector& Vector::normalize() {
	return (*this) /= length();
}

Vector Vector::cross(const Vector& o) const {
	return Vector{
		y*o.z - z*o.y,
			z*o.x - x*o.z,
			x*o.y - y*o.x
	};
}

float Vector::dot(const Vector& o) const {
	return x*o.x + y*o.y + z*o.z;
}

// makes the vector perpendicular to o
// if the vector is collinear with o, it is left unchanged
// and the function returns false, otherwise returns true
bool Vector::perpendicularize(Vector o) {
	o.normalize();
	Vector self = *this;
	self.normalize();
	if(fabs(self.dot(o)) == 1) {
		return false;
	}
	else {
		Vector c = o.cross(self);
		*this = c.cross(o);
		return true;
	}
}

std::ostream& operator<<(std::ostream& o, const Vector& v) {
	return o << v.x << " \t" << v.y << " \t" << v.z;
}
