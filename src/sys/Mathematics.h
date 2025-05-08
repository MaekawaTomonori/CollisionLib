#pragma once
#define NOMINMAX
#include <iostream>

namespace Collision {// 3次元ベクトル
    class Vec3{
        public:
        float x, y, z;

        Vec3();

        Vec3(float x, float y, float z);

        Vec3 operator+(const Vec3& other) const;

        Vec3 operator-(const Vec3& other) const;

        Vec3 operator-(const float other) const;

        Vec3 operator*(float scalar) const;

        Vec3 operator/(float scalar) const;

        Vec3& operator+=(const Vec3& other);

        Vec3& operator-=(const Vec3& other);

        Vec3& operator*=(float scalar);

        Vec3& operator/=(float scalar);

        bool operator==(const Vec3& other) const;

        bool operator!=(const Vec3& other) const;

        float Length() const;

        float SquaredLength() const;

        Vec3 Normalized() const;

        void Normalize();

        static float Dot(const Vec3& a, const Vec3& b);

        static Vec3 Cross(const Vec3& a, const Vec3& b);

        static Vec3 Lerp(const Vec3& a, const Vec3& b, float t);

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

        Vec3i();

        Vec3i(int x, int y, int z);

        bool operator==(const Vec3i& other) const;

        bool operator!=(const Vec3i& other) const;
    };

    // 外部ストリーム出力演算子
    inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
}

