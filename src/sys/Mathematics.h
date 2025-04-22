#pragma once
#define NOMINMAX
#include <iostream>
#include <variant>

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

        float operator[](int index) const;

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

	class Quat{
        public:
        float x, y, z, w;

        // コンストラクタ
        Quat();
        Quat(float x, float y, float z, float w);
        Quat(const Vec3& axis, float angle);

        // 単位四元数
        static const Quat Identity;

        // 基本演算
        Quat operator*(const Quat& other) const;
        Quat& operator*=(const Quat& other);
        Quat operator*(float scalar) const;
        Quat operator/(float scalar) const;
        Quat& operator*=(float scalar);
        Quat& operator/=(float scalar);
        Quat operator+(const Quat& other) const;
        Quat operator-(const Quat& other) const;
        bool operator==(const Quat& other) const;
        bool operator!=(const Quat& other) const;

        // ノルム・正規化関連
        float Length() const;
        float SquaredLength() const;
        Quat Normalized() const;
        void Normalize();
        Quat Conjugate() const;
        Quat Inverse() const;

        // 回転関連
        Vec3 RotateVector(const Vec3& v) const;
        static Quat FromEulerAngles(float pitch, float yaw, float roll);
        static Quat FromAxisAngle(const Vec3& axis, float angle);
        static Quat Lerp(const Quat& a, const Quat& b, float t);
        static Quat Slerp(const Quat& a, const Quat& b, float t);
    };

    struct Transform{
        Vec3 translate;
        std::variant<Vec3, Quat> rotation;
        std::variant<float, Vec3> scale;
    };

    // 外部ストリーム出力演算子
    inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
}

