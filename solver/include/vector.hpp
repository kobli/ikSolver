#ifndef VECTOR_HPP_18_12_18_14_17_20
#define VECTOR_HPP_18_12_18_14_17_20 
#include <cmath>

class Vector {
	public:
		Vector(float v = 0): x{v}, y{v}, z{v}
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

		Vector cross(const Vector& o) const {
			return Vector{
				y*o.z - z*o.y,
				z*o.x - x*o.z,
				x*o.y - y*o.x
			};
		}

		float dot(const Vector& o) const {
			return x*o.x + y*o.y + z*o.z;
		}

		// makes the vector perpendicular to o
		// if the vector is collinear with o, it is left unchanged
		// and the function returns false, otherwise returns true
		bool perpendicularize(Vector o) {
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

		float x;
		float y;
		float z;
};
#endif /* VECTOR_HPP_18_12_18_14_17_20 */
