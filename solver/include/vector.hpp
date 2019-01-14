#ifndef VECTOR_HPP_18_12_18_14_17_20
#define VECTOR_HPP_18_12_18_14_17_20 
#include <cmath>
#include <ostream>

class Vector {
	public:
		Vector(float v = 0);
		Vector(float X, float Y, float Z);
		Vector operator+(const Vector& o) const;
		Vector operator-(const Vector& o) const;
		Vector operator*(float v) const;
		Vector& operator/=(float v);
		float length() const;
		Vector& normalize();
		Vector cross(const Vector& o) const;
		float dot(const Vector& o) const;

		// makes the vector perpendicular to o
		// if the vector is collinear with o, it is left unchanged
		// and the function returns false, otherwise returns true
		bool perpendicularize(Vector o);

		float x;
		float y;
		float z;
};

std::ostream& operator<<(std::ostream& o, const Vector& v);
#endif /* VECTOR_HPP_18_12_18_14_17_20 */
