#include "Mathematics.h"

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
	    return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 Vec3::operator-(const Vec3& other) const {
	    return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 Vec3::operator-(const float other) const {
	    return Vec3(x - other, y - other, z - other);
    }

    Vec3 Vec3::operator*(float scalar) const {
	    return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 Vec3::operator/(float scalar) const {
	    return Vec3(x / scalar, y / scalar, z / scalar);
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

    float Vec3::operator[](int index) const {
        if (index == 0) return x;
        if (index == 1) return y;
        if (index == 2) return z;
        throw std::out_of_range("Index out of range");
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
    Quat::Quat() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {
    }

    Quat::Quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {
    }

    Quat::Quat(const Vec3& axis, float angle) {
        Vec3 normalizedAxis = axis;
        normalizedAxis.Normalize();

        float halfAngle = angle * 0.5f;
        float sinHalfAngle = std::sin(halfAngle);

        x = normalizedAxis.x * sinHalfAngle;
        y = normalizedAxis.y * sinHalfAngle;
        z = normalizedAxis.z * sinHalfAngle;
        w = std::cos(halfAngle);
    }

    Quat Quat::operator*(const Quat& other) const {
        return {
	        w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w,
            w * other.w - x * other.x - y * other.y - z * other.z
        };
    }

    Quat& Quat::operator*=(const Quat& other) {
        *this = *this * other;
        return *this;
    }

    Quat Quat::operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar, w * scalar};
    }

    Quat Quat::operator/(float scalar) const {
        return {x / scalar, y / scalar, z / scalar, w / scalar};
    }

    Quat& Quat::operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        w /= scalar;
        return *this;
    }

    Quat Quat::operator+(const Quat& other) const {
        return {x + other.x, y + other.y, z + other.z, w + other.w};
    }

    Quat Quat::operator-(const Quat& other) const {
        return {x - other.x, y - other.y, z - other.z, w - other.w};
    }

    bool Quat::operator==(const Quat& other) const {
        return x == other.x && y == other.y && z == other.z && w == other.w;
    }

    bool Quat::operator!=(const Quat& other) const {
        return !(*this == other);
    }

    float Quat::Length() const {
        return std::sqrt(x * x + y * y + z * z + w * w);
    }

    float Quat::SquaredLength() const {
        return x * x + y * y + z * z + w * w;
    }

    Quat Quat::Normalized() const {
        float len = Length();
        if (len < 0.0001f){
            return Identity;
        }
        return *this / len;
    }

    void Quat::Normalize() {
        float len = Length();
        if (len >= 0.0001f){
            *this /= len;
        } else{
            *this = Identity;
        }
    }

    Quat Quat::Conjugate() const {
        return {-x, -y, -z, w};
    }

    Quat Quat::Inverse() const {
        float sqLen = SquaredLength();
        if (sqLen < 0.0001f){
            return Identity;
        }
        return Conjugate() / sqLen;
    }

    Vec3 Quat::RotateVector(const Vec3& v) const {
        // q * v * q^-1
        Quat vecQuat(v.x, v.y, v.z, 0.0f);
        Quat result = *this * vecQuat * Conjugate();
        return {result.x, result.y, result.z};
    }

    Quat Quat::FromEulerAngles(float pitch, float yaw, float roll) {
        // ピッチ(X)、ヨー(Y)、ロール(Z)からクォータニオンを生成
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);

        Quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    Quat Quat::FromAxisAngle(const Vec3& axis, float angle) {
        return Quat(axis, angle);
    }

    Quat Quat::Lerp(const Quat& a, const Quat& b, float t) {
        // 線形補間
        Quat result;
        float t_ = 1.0f - t;

        result.x = t_ * a.x + t * b.x;
        result.y = t_ * a.y + t * b.y;
        result.z = t_ * a.z + t * b.z;
        result.w = t_ * a.w + t * b.w;

        return result.Normalized();
    }

    Quat Quat::Slerp(const Quat& a, const Quat& b, float t) {
        // 球面線形補間
        Quat q1 = a;
        Quat q2 = b;

        float dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

        // 最短経路を取るための調整
        if (dot < 0.0f){
            q2 = Quat(-q2.x, -q2.y, -q2.z, -q2.w);
            dot = -dot;
        }

        // 非常に近い場合は線形補間を使用
        if (dot > 0.9995f){
            return Lerp(q1, q2, t);
        }

        // 実際のSlerp
        float angle = std::acos(dot);
        float sinAngle = std::sin(angle);

        float t1 = std::sin((1.0f - t) * angle) / sinAngle;
        float t2 = std::sin(t * angle) / sinAngle;

        return {
	        q1.x * t1 + q2.x * t2,
            q1.y * t1 + q2.y * t2,
            q1.z * t1 + q2.z * t2,
            q1.w * t1 + q2.w * t2
        };
    }
}
