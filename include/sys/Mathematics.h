#pragma once
#include <cmath>
#include <algorithm>
#include <array>
#include <iostream>
#include <numbers>

namespace Collision {// 3次元ベクトル
    class Vector3{
        public:
        float x, y, z;

        Vector3() : x(0.0f), y(0.0f), z(0.0f) {
        }
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {
        }

        Vector3 operator+(const Vector3& other) const {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }

        Vector3 operator-(const Vector3& other) const {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }

        Vector3 operator-(const float other) const {
            return Vector3(x - other, y - other, z - other);
        }

        Vector3 operator*(float scalar) const {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }

        Vector3 operator/(float scalar) const {
            return Vector3(x / scalar, y / scalar, z / scalar);
        }

        Vector3& operator+=(const Vector3& other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        Vector3& operator-=(const Vector3& other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        Vector3& operator*=(float scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        Vector3& operator/=(float scalar) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        bool operator==(const Vector3& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const Vector3& other) const {
            return !(*this == other);
        }

        float Length() const {
            return std::sqrt(x * x + y * y + z * z);
        }

        float SquaredLength() const {
            return x * x + y * y + z * z;
        }

        Vector3 Normalized() const {
            float len = Length();
            if (len < 0.0001f){
                return Vector3(0, 0, 0);
            }
            return *this / len;
        }

        void Normalize() {
            float len = Length();
            if (len >= 0.0001f){
                *this /= len;
            }
        }

        static float Dot(const Vector3& a, const Vector3& b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        static Vector3 Cross(const Vector3& a, const Vector3& b) {
            return Vector3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }

        static Vector3 Lerp(const Vector3& a, const Vector3& b, float t) {
            return a + (b - a) * t;
        }

        static const Vector3 Zero;
        static const Vector3 One;
        static const Vector3 Up;
        static const Vector3 Down;
        static const Vector3 Left;
        static const Vector3 Right;
        static const Vector3 Forward;
        static const Vector3 Backward;
    };

    // 整数型3次元ベクトル
    struct Vector3i{
        int x, y, z;

        Vector3i() : x(0), y(0), z(0) {
        }
        Vector3i(int x, int y, int z) : x(x), y(y), z(z) {
        }

        bool operator==(const Vector3i& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const Vector3i& other) const {
            return !(*this == other);
        }
    };

    // 4x4行列
    class Matrix4x4{
        public:
        float m[16];  // 行優先

        Matrix4x4() {
            Identity();
        }

        Matrix4x4(const float* values) {
            std::copy(values, values + 16, m);
        }

        void Identity() {
            std::fill(m, m + 16, 0.0f);
            m[0] = m[5] = m[10] = m[15] = 1.0f;
        }

        Matrix4x4 operator*(const Matrix4x4& other) const {
            Matrix4x4 result;

            for (int i = 0; i < 4; ++i){
                for (int j = 0; j < 4; ++j){
                    result.m[i * 4 + j] =
                        m[i * 4 + 0] * other.m[0 * 4 + j] +
                        m[i * 4 + 1] * other.m[1 * 4 + j] +
                        m[i * 4 + 2] * other.m[2 * 4 + j] +
                        m[i * 4 + 3] * other.m[3 * 4 + j];
                }
            }

            return result;
        }

        Vector3 TransformPoint(const Vector3& point) const {
            float x = point.x * m[0] + point.y * m[4] + point.z * m[8] + m[12];
            float y = point.x * m[1] + point.y * m[5] + point.z * m[9] + m[13];
            float z = point.x * m[2] + point.y * m[6] + point.z * m[10] + m[14];
            float w = point.x * m[3] + point.y * m[7] + point.z * m[11] + m[15];

            if (std::abs(w) < 0.0001f){
                return Vector3(x, y, z);
            }

            return Vector3(x / w, y / w, z / w);
        }

        Vector3 TransformVector(const Vector3& vector) const {
            float x = vector.x * m[0] + vector.y * m[4] + vector.z * m[8];
            float y = vector.x * m[1] + vector.y * m[5] + vector.z * m[9];
            float z = vector.x * m[2] + vector.y * m[6] + vector.z * m[10];

            return Vector3(x, y, z);
        }

        static Matrix4x4 Translate(const Vector3& translation) {
            Matrix4x4 result;

            result.m[12] = translation.x;
            result.m[13] = translation.y;
            result.m[14] = translation.z;

            return result;
        }

        static Matrix4x4 Scale(const Vector3& scale) {
            Matrix4x4 result;

            result.m[0] = scale.x;
            result.m[5] = scale.y;
            result.m[10] = scale.z;

            return result;
        }


        static Matrix4x4 Inverse(const Matrix4x4& matrix) {
            // ここでは簡単のため、TRS行列の逆行列のみをサポート
            // 一般的な4x4行列の逆行列計算は複雑なため省略

            Matrix4x4 result;
            // ...
            return result;
        }
    };

    // AABB (Axis-Aligned Bounding Box)
    struct AABB{
        Vector3 min;
        Vector3 max;

        AABB()
            : min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
            max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()) {
        }

        AABB(const Vector3& min, const Vector3& max)
            : min(min), max(max) {
        }

        Vector3 GetCenter() const {
            return (min + max) * 0.5f;
        }

        Vector3 GetExtents() const {
            return (max - min) * 0.5f;
        }

        Vector3 GetSize() const {
            return max - min;
        }

        float GetVolume() const {
            Vector3 size = GetSize();
            return size.x * size.y * size.z;
        }

        bool Contains(const Vector3& point) const {
            return point.x >= min.x && point.x <= max.x &&
                point.y >= min.y && point.y <= max.y &&
                point.z >= min.z && point.z <= max.z;
        }

        bool Intersects(const AABB& other) const {
            return min.x <= other.max.x && max.x >= other.min.x &&
                min.y <= other.max.y && max.y >= other.min.y &&
                min.z <= other.max.z && max.z >= other.min.z;
        }

        void Expand(const Vector3& point) {
            min.x = std::min(min.x, point.x);
            min.y = std::min(min.y, point.y);
            min.z = std::min(min.z, point.z);

            max.x = std::max(max.x, point.x);
            max.y = std::max(max.y, point.y);
            max.z = std::max(max.z, point.z);
        }

        void Expand(const AABB& other) {
            min.x = std::min(min.x, other.min.x);
            min.y = std::min(min.y, other.min.y);
            min.z = std::min(min.z, other.min.z);

            max.x = std::max(max.x, other.max.x);
            max.y = std::max(max.y, other.max.y);
            max.z = std::max(max.z, other.max.z);
        }

        static AABB FromCenterExtents(const Vector3& center, const Vector3& extents) {
            return AABB(center - extents, center + extents);
        }
    };

    // Ray（レイ）
    struct Ray{
        Vector3 origin;
        Vector3 direction;

        Ray() : origin(0.0f, 0.0f, 0.0f), direction(0.0f, 0.0f, 1.0f) {
        }
        Ray(const Vector3& origin, const Vector3& direction)
            : origin(origin), direction(direction.Normalized()) {
        }

        Vector3 GetPoint(float distance) const {
            return origin + direction * distance;
        }

        bool Intersects(const AABB& aabb, float& distance) const {
            float tmin = 0.0f;
            float tmax = std::numeric_limits<float>::max();

            for (int i = 0; i < 3; ++i){
                float component = i == 0 ? direction.x : (i == 1 ? direction.y : direction.z);

                if (std::abs(component) < 0.0001f){
                    float min = i == 0 ? aabb.min.x : (i == 1 ? aabb.min.y : aabb.min.z);
                    float max = i == 0 ? aabb.max.x : (i == 1 ? aabb.max.y : aabb.max.z);
                    float orig = i == 0 ? origin.x : (i == 1 ? origin.y : origin.z);

                    if (orig < min || orig > max){
                        return false;
                    }
                } else{
                    float invD = 1.0f / component;
                    float min = i == 0 ? aabb.min.x : (i == 1 ? aabb.min.y : aabb.min.z);
                    float max = i == 0 ? aabb.max.x : (i == 1 ? aabb.max.y : aabb.max.z);
                    float orig = i == 0 ? origin.x : (i == 1 ? origin.y : origin.z);

                    float t1 = (min - orig) * invD;
                    float t2 = (max - orig) * invD;

                    if (t1 > t2){
                        std::swap(t1, t2);
                    }

                    tmin = std::max(tmin, t1);
                    tmax = std::min(tmax, t2);

                    if (tmin > tmax){
                        return false;
                    }
                }
            }

            distance = tmin;
            return true;
        }
    };

    // 外部ストリーム出力演算子
    inline std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
        os << "AABB{min=" << aabb.min << ", max=" << aabb.max << "}";
        return os;
    }
}

