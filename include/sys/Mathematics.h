#pragma once
#include <cmath>
#include <algorithm>
#include <array>
#include <iostream>
#include <numbers>

namespace Collision {// 3次元ベクトル
    class Vec3{
        public:
        float x, y, z;

        Vec3() : x(0.0f), y(0.0f), z(0.0f) {
        }
        Vec3(float x, float y, float z) : x(x), y(y), z(z) {
        }

        Vec3 operator+(const Vec3& other) const {
            return Vec3(x + other.x, y + other.y, z + other.z);
        }

        Vec3 operator-(const Vec3& other) const {
            return Vec3(x - other.x, y - other.y, z - other.z);
        }

        Vec3 operator-(const float other) const {
            return Vec3(x - other, y - other, z - other);
        }

        Vec3 operator*(float scalar) const {
            return Vec3(x * scalar, y * scalar, z * scalar);
        }

        Vec3 operator/(float scalar) const {
            return Vec3(x / scalar, y / scalar, z / scalar);
        }

        Vec3& operator+=(const Vec3& other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        Vec3& operator-=(const Vec3& other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        Vec3& operator*=(float scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        Vec3& operator/=(float scalar) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        bool operator==(const Vec3& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const Vec3& other) const {
            return !(*this == other);
        }

        float Length() const {
            return std::sqrt(x * x + y * y + z * z);
        }

        float SquaredLength() const {
            return x * x + y * y + z * z;
        }

        Vec3 Normalized() const {
            float len = Length();
            if (len < 0.0001f){
                return Vec3(0, 0, 0);
            }
            return *this / len;
        }

        void Normalize() {
            float len = Length();
            if (len >= 0.0001f){
                *this /= len;
            }
        }

        static float Dot(const Vec3& a, const Vec3& b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        static Vec3 Cross(const Vec3& a, const Vec3& b) {
            return Vec3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }

        static Vec3 Lerp(const Vec3& a, const Vec3& b, float t) {
            return a + (b - a) * t;
        }

        static const Vec3 Zero;
        static const Vec3 One;
        static const Vec3 Up;
        static const Vec3 Down;
        static const Vec3 Left;
        static const Vec3 Right;
        static const Vec3 Forward;
        static const Vec3 Backward;
    };

    // 整数型3次元ベクトル
    struct Vec3i{
        int x, y, z;

        Vec3i() : x(0), y(0), z(0) {
        }
        Vec3i(int x, int y, int z) : x(x), y(y), z(z) {
        }

        bool operator==(const Vec3i& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const Vec3i& other) const {
            return !(*this == other);
        }
    };

    // 4x4行列
    class Mat4x4{
        public:
        float m[16];  // 行優先

        Mat4x4() {
            Identity();
        }

        Mat4x4(const float* values) {
            std::copy(values, values + 16, m);
        }

        void Identity() {
            std::fill(m, m + 16, 0.0f);
            m[0] = m[5] = m[10] = m[15] = 1.0f;
        }

        Mat4x4 operator*(const Mat4x4& other) const {
            Mat4x4 result;

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

        Vec3 TransformPoint(const Vec3& point) const {
            float x = point.x * m[0] + point.y * m[4] + point.z * m[8] + m[12];
            float y = point.x * m[1] + point.y * m[5] + point.z * m[9] + m[13];
            float z = point.x * m[2] + point.y * m[6] + point.z * m[10] + m[14];
            float w = point.x * m[3] + point.y * m[7] + point.z * m[11] + m[15];

            if (std::abs(w) < 0.0001f){
                return Vec3(x, y, z);
            }

            return Vec3(x / w, y / w, z / w);
        }

        Vec3 TransformVector(const Vec3& vector) const {
            float x = vector.x * m[0] + vector.y * m[4] + vector.z * m[8];
            float y = vector.x * m[1] + vector.y * m[5] + vector.z * m[9];
            float z = vector.x * m[2] + vector.y * m[6] + vector.z * m[10];

            return Vec3(x, y, z);
        }

        static Mat4x4 Translate(const Vec3& translation) {
            Mat4x4 result;

            result.m[12] = translation.x;
            result.m[13] = translation.y;
            result.m[14] = translation.z;

            return result;
        }

        static Mat4x4 Scale(const Vec3& scale) {
            Mat4x4 result;

            result.m[0] = scale.x;
            result.m[5] = scale.y;
            result.m[10] = scale.z;

            return result;
        }


        static Mat4x4 Inverse(const Mat4x4& matrix) {
            // ここでは簡単のため、TRS行列の逆行列のみをサポート
            // 一般的な4x4行列の逆行列計算は複雑なため省略

            Mat4x4 result;
            // ...
            return result;
        }
    };

    // AABB (Axis-Aligned Bounding Box)
    struct AABB{
        Vec3 min;
        Vec3 max;

        AABB()
            : min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
            max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()) {
        }

        AABB(const Vec3& min, const Vec3& max)
            : min(min), max(max) {
        }

        Vec3 GetCenter() const {
            return (min + max) * 0.5f;
        }

        Vec3 GetExtents() const {
            return (max - min) * 0.5f;
        }

        Vec3 GetSize() const {
            return max - min;
        }

        float GetVolume() const {
            Vec3 size = GetSize();
            return size.x * size.y * size.z;
        }

        bool Contains(const Vec3& point) const {
            return point.x >= min.x && point.x <= max.x &&
                point.y >= min.y && point.y <= max.y &&
                point.z >= min.z && point.z <= max.z;
        }

        bool Intersects(const AABB& other) const {
            return min.x <= other.max.x && max.x >= other.min.x &&
                min.y <= other.max.y && max.y >= other.min.y &&
                min.z <= other.max.z && max.z >= other.min.z;
        }

        void Expand(const Vec3& point) {
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

        static AABB FromCenterExtents(const Vec3& center, const Vec3& extents) {
            return AABB(center - extents, center + extents);
        }
    };

    // Ray（レイ）
    struct Ray{
        Vec3 origin;
        Vec3 direction;

        Ray() : origin(0.0f, 0.0f, 0.0f), direction(0.0f, 0.0f, 1.0f) {
        }
        Ray(const Vec3& origin, const Vec3& direction)
            : origin(origin), direction(direction.Normalized()) {
        }

        Vec3 GetPoint(float distance) const {
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
    inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
        os << "AABB{min=" << aabb.min << ", max=" << aabb.max << "}";
        return os;
    }
}

