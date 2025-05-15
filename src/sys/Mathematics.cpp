#include "Collision/Mathematics.h"

namespace Collision {
    // 静的メンバの定義
    const Vec3 Vec3::Zero = Vec3(0.0f, 0.0f, 0.0f);
    const Vec3 Vec3::One = Vec3(1.0f, 1.0f, 1.0f);
    const Vec3 Vec3::Up = Vec3(0.0f, 1.0f, 0.0f);
    const Vec3 Vec3::Down = Vec3(0.0f, -1.0f, 0.0f);
    const Vec3 Vec3::Left = Vec3(-1.0f, 0.0f, 0.0f);
    const Vec3 Vec3::Right = Vec3(1.0f, 0.0f, 0.0f);
    const Vec3 Vec3::Forward = Vec3(0.0f, 0.0f, 1.0f);
    const Vec3 Vec3::Backward = Vec3(0.0f, 0.0f, -1.0f);

    Vec3::Vec3(): x(0.0f), y(0.0f), z(0.0f) {
    }

    Vec3::Vec3(float x, float y, float z): x(x), y(y), z(z) {
    }

    Vec3 Vec3::operator+(const Vec3& other) const {
	    return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 Vec3::operator-(const Vec3& other) const {
	    return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 Vec3::operator-(const float other) const {
	    return {x - other, y - other, z - other};
    }

    Vec3 Vec3::operator*(float scalar) const {
	    return {x * scalar, y * scalar, z * scalar};
    }

    Vec3 Vec3::operator/(float scalar) const {
	    return {x / scalar, y / scalar, z / scalar};
    }

    Vec3 Vec3::operator/(const Vec3& other) const {
        return {x / other.x, y / other.y, z / other.z};
    }

    Vec3& Vec3::operator+=(const Vec3& other) {
	    x += other.x;
	    y += other.y;
	    z += other.z;
	    return *this;
    }

    Vec3& Vec3::operator-=(const Vec3& other) {
	    x -= other.x;
	    y -= other.y;
	    z -= other.z;
	    return *this;
    }

    Vec3& Vec3::operator*=(float scalar) {
	    x *= scalar;
	    y *= scalar;
	    z *= scalar;
	    return *this;
    }

    Vec3& Vec3::operator/=(float scalar) {
	    x /= scalar;
	    y /= scalar;
	    z /= scalar;
	    return *this;
    }

    bool Vec3::operator==(const Vec3& other) const {
	    return x == other.x && y == other.y && z == other.z;
    }

    bool Vec3::operator!=(const Vec3& other) const {
	    return !(*this == other);
    }

    float Vec3::Length() const {
	    return std::sqrt(x * x + y * y + z * z);
    }

    float Vec3::SquaredLength() const {
	    return x * x + y * y + z * z;
    }

    Vec3 Vec3::Normalized() const {
	    float len = Length();
	    if (len < 0.0001f){
		    return Vec3(0, 0, 0);
	    }
	    return *this / len;
    }

    void Vec3::Normalize() {
	    float len = Length();
	    if (len >= 0.0001f){
		    *this /= len;
	    }
    }

    float Vec3::Dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    float Vec3::Dot(const Vec3& a, const Vec3& b) {
	    return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Vec3 Vec3::Cross(const Vec3& a, const Vec3& b) {
	    return Vec3(
		    a.y * b.z - a.z * b.y,
		    a.z * b.x - a.x * b.z,
		    a.x * b.y - a.y * b.x
	    );
    }

    Vec3 Vec3::Lerp(const Vec3& a, const Vec3& b, float t) {
	    return a + (b - a) * t;
    }

    Vec3i::Vec3i(): x(0), y(0), z(0) {
    }

    Vec3i::Vec3i(int x, int y, int z): x(x), y(y), z(z) {
    }

    bool Vec3i::operator==(const Vec3i& other) const {
	    return x == other.x && y == other.y && z == other.z;
    }

    bool Vec3i::operator!=(const Vec3i& other) const {
	    return !(*this == other);
    }
}
