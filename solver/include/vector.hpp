#ifndef VECTOR_HPP_18_12_18_14_17_20
#define VECTOR_HPP_18_12_18_14_17_20 
#include <cmath>

class Vector {
	public:
		Vector(float v): x{v}, y{v}, z{v}
		{}

		Vector(float X, float Y, float Z): x{X}, y{Y}, z{Z}
		{}

		Vector operator+(const Vector& o) const {
			return Vector(x+o.x, y+o.y, z+o.z);
		}

		Vector operator-(const Vector& o) const {
			return Vector(x-o.x, y-o.y, z-o.z);
		}

		Vector operator*(float v) const {
			return Vector(x*v, y*v, z*v);
		}

		Vector& operator/=(float v) {
			x /= v;
			y /= v;
			z /= v;
			return *this;
		}

		float length() const {
			return std::sqrt(x*x + y*y + z*z);
		}

		Vector& normalize() {
			return (*this) /= length();
		}

		float x;
		float y;
		float z;
};
#endif /* VECTOR_HPP_18_12_18_14_17_20 */
