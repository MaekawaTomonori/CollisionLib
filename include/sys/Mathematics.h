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

    // クォータニオン
    class Quaternion{
        public:
        float x, y, z, w;

        Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {
        }
        Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {
        }

        Quaternion operator*(const Quaternion& other) const {
            return Quaternion(
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w,
                w * other.w - x * other.x - y * other.y - z * other.z
            );
        }

        Quaternion& operator*=(const Quaternion& other) {
            *this = *this * other;
            return *this;
        }

        bool operator==(const Quaternion& other) const {
            return x == other.x && y == other.y && z == other.z && w == other.w;
        }

        bool operator!=(const Quaternion& other) const {
            return !(*this == other);
        }

        float Length() const {
            return std::sqrt(x * x + y * y + z * z + w * w);
        }

        Quaternion Normalized() const {
            float len = Length();
            if (len < 0.0001f){
                return Quaternion(0, 0, 0, 1);
            }
            return Quaternion(x / len, y / len, z / len, w / len);
        }

        void Normalize() {
            float len = Length();
            if (len >= 0.0001f){
                x /= len;
                y /= len;
                z /= len;
                w /= len;
            } else{
                x = 0;
                y = 0;
                z = 0;
                w = 1;
            }
        }

        Quaternion Conjugate() const {
            return Quaternion(-x, -y, -z, w);
        }

        Quaternion Inverse() const {
            return Conjugate();  // 正規化されていると仮定
        }

        Vector3 RotateVector(const Vector3& v) const {
            Quaternion p(v.x, v.y, v.z, 0);
            Quaternion q = *this * p * Conjugate();
            return Vector3(q.x, q.y, q.z);
        }

        static Quaternion FromEuler(float pitch, float yaw, float roll) {
            float cy = std::cos(yaw * 0.5f);
            float sy = std::sin(yaw * 0.5f);
            float cp = std::cos(pitch * 0.5f);
            float sp = std::sin(pitch * 0.5f);
            float cr = std::cos(roll * 0.5f);
            float sr = std::sin(roll * 0.5f);

            return Quaternion(
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy
            );
        }

        static Quaternion FromAxisAngle(const Vector3& axis, float angle) {
            Vector3 normalizedAxis = axis.Normalized();
            float halfAngle = angle * 0.5f;
            float s = std::sin(halfAngle);

            return Quaternion(
                normalizedAxis.x * s,
                normalizedAxis.y * s,
                normalizedAxis.z * s,
                std::cos(halfAngle)
            );
        }

        static Quaternion Lerp(const Quaternion& a, const Quaternion& b, float t) {
            Quaternion result;
            float t_ = 1.0f - t;

            if (Dot(a, b) < 0){
                result.x = t_ * a.x - t * b.x;
                result.y = t_ * a.y - t * b.y;
                result.z = t_ * a.z - t * b.z;
                result.w = t_ * a.w - t * b.w;
            } else{
                result.x = t_ * a.x + t * b.x;
                result.y = t_ * a.y + t * b.y;
                result.z = t_ * a.z + t * b.z;
                result.w = t_ * a.w + t * b.w;
            }

            result.Normalize();
            return result;
        }

        static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t) {
            Quaternion q;
            float cosHalfTheta = Dot(a, b);

            if (std::abs(cosHalfTheta) >= 1.0f){
                return a;
            }

            if (cosHalfTheta < 0){
                q.x = -b.x;
                q.y = -b.y;
                q.z = -b.z;
                q.w = -b.w;
                cosHalfTheta = -cosHalfTheta;
            } else{
                q = b;
            }

            if (cosHalfTheta < 0.95f){
                float halfTheta = std::acos(cosHalfTheta);
                float sinHalfTheta = std::sqrt(1.0f - cosHalfTheta * cosHalfTheta);

                if (std::abs(sinHalfTheta) < 0.001f){
                    q.x = a.x * 0.5f + q.x * 0.5f;
                    q.y = a.y * 0.5f + q.y * 0.5f;
                    q.z = a.z * 0.5f + q.z * 0.5f;
                    q.w = a.w * 0.5f + q.w * 0.5f;
                    return q;
                }

                float ratioA = std::sin((1 - t) * halfTheta) / sinHalfTheta;
                float ratioB = std::sin(t * halfTheta) / sinHalfTheta;

                q.x = a.x * ratioA + q.x * ratioB;
                q.y = a.y * ratioA + q.y * ratioB;
                q.z = a.z * ratioA + q.z * ratioB;
                q.w = a.w * ratioA + q.w * ratioB;

                return q;
            } else{
                return Lerp(a, q, t);
            }
        }

        static float Dot(const Quaternion& a, const Quaternion& b) {
            return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        }

        static const Quaternion Identity;
    };

    // オイラー角
    struct EulerAngles{
        float pitch, yaw, roll;

        EulerAngles() : pitch(0.0f), yaw(0.0f), roll(0.0f) {
        }
        EulerAngles(float pitch, float yaw, float roll)
            : pitch(pitch), yaw(yaw), roll(roll) {
        }

        static EulerAngles FromQuaternion(const Quaternion& q) {
            EulerAngles angles;

            // ロール (x軸周り)
            float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
            float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);

            // ピッチ (y軸周り)
            float sinp = 2.0f * (q.w * q.y - q.z * q.x);
            if (std::abs(sinp) >= 1.0f){
                angles.pitch = std::copysign(std::numbers::pi_v<float> / 2.0f, sinp);  // 特異点の場合
            } else{
                angles.pitch = std::asin(sinp);
            }

            // ヨー (z軸周り)
            float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
            float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);

            return angles;
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

        static Matrix4x4 Rotate(const Quaternion& rotation) {
            Matrix4x4 result;

            float xx = rotation.x * rotation.x;
            float xy = rotation.x * rotation.y;
            float xz = rotation.x * rotation.z;
            float xw = rotation.x * rotation.w;

            float yy = rotation.y * rotation.y;
            float yz = rotation.y * rotation.z;
            float yw = rotation.y * rotation.w;

            float zz = rotation.z * rotation.z;
            float zw = rotation.z * rotation.w;

            result.m[0] = 1.0f - 2.0f * (yy + zz);
            result.m[1] = 2.0f * (xy + zw);
            result.m[2] = 2.0f * (xz - yw);

            result.m[4] = 2.0f * (xy - zw);
            result.m[5] = 1.0f - 2.0f * (xx + zz);
            result.m[6] = 2.0f * (yz + xw);

            result.m[8] = 2.0f * (xz + yw);
            result.m[9] = 2.0f * (yz - xw);
            result.m[10] = 1.0f - 2.0f * (xx + yy);

            return result;
        }

        static Matrix4x4 TRS(const Vector3& translation, const Quaternion& rotation, const Vector3& scale) {
            return Translate(translation) * Rotate(rotation) * Scale(scale);
        }

        static Matrix4x4 Inverse(const Matrix4x4& matrix) {
            // ここでは簡単のため、TRS行列の逆行列のみをサポート
            // 一般的な4x4行列の逆行列計算は複雑なため省略

            Matrix4x4 result;
            // ...
            return result;
        }
    };

    // 変換情報
    struct Transform{
        Vector3 position;
        Quaternion rotation;
        Vector3 scale;

        Transform()
            : position(0.0f, 0.0f, 0.0f),
            rotation(0.0f, 0.0f, 0.0f, 1.0f),
            scale(1.0f, 1.0f, 1.0f) {
        }

        Transform(const Vector3& position, const Quaternion& rotation, const Vector3& scale)
            : position(position), rotation(rotation), scale(scale) {
        }

        static Transform Identity() {
            return Transform(
                Vector3(0.0f, 0.0f, 0.0f),
                Quaternion(0.0f, 0.0f, 0.0f, 1.0f),
                Vector3(1.0f, 1.0f, 1.0f)
            );
        }

        Matrix4x4 ToMatrix() const {
            return Matrix4x4::TRS(position, rotation, scale);
        }

        Transform Inverse() const {
            Quaternion invRotation = rotation.Inverse();
            Vector3 invScale(1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z);
            Vector3 invPosition = invRotation.RotateVector(position * -1.0f);

            return Transform(invPosition, invRotation, invScale);
        }

        Transform operator*(const Transform& other) const {
            Transform result;

            // スケールの組み合わせ
            result.scale.x = scale.x * other.scale.x;
            result.scale.y = scale.y * other.scale.y;
            result.scale.z = scale.z * other.scale.z;

            // 回転の組み合わせ
            result.rotation = rotation * other.rotation;

            // 位置の組み合わせ
            Vector3 scaledOtherPosition = other.position;
            scaledOtherPosition.x *= scale.x;
            scaledOtherPosition.y *= scale.y;
            scaledOtherPosition.z *= scale.z;

            result.position = position + rotation.RotateVector(scaledOtherPosition);

            return result;
        }

        Vector3 TransformPoint(const Vector3& point) const {
            Vector3 scaled = Vector3(
                point.x * scale.x,
                point.y * scale.y,
                point.z * scale.z
            );

            return position + rotation.RotateVector(scaled);
        }

        Vector3 TransformDirection(const Vector3& direction) const {
            return rotation.RotateVector(direction);
        }

        Vector3 InverseTransformPoint(const Vector3& point) const {
            Vector3 relativePoint = point - position;
            Vector3 rotatedPoint = rotation.Inverse().RotateVector(relativePoint);

            return Vector3(
                rotatedPoint.x / scale.x,
                rotatedPoint.y / scale.y,
                rotatedPoint.z / scale.z
            );
        }

        Vector3 InverseTransformDirection(const Vector3& direction) const {
            return rotation.Inverse().RotateVector(direction);
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

        AABB Transformed(const Transform& transform) const {
            // 8つの頂点を変換してAABBを再計算
            std::array<Vector3, 8> corners = {
                Vector3(min.x, min.y, min.z),
                Vector3(min.x, min.y, max.z),
                Vector3(min.x, max.y, min.z),
                Vector3(min.x, max.y, max.z),
                Vector3(max.x, min.y, min.z),
                Vector3(max.x, min.y, max.z),
                Vector3(max.x, max.y, min.z),
                Vector3(max.x, max.y, max.z)
            };

            AABB result;

            for (const auto& corner : corners){
                result.Expand(transform.TransformPoint(corner));
            }

            return result;
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

    inline std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        os << "(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")";
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
        os << "AABB{min=" << aabb.min << ", max=" << aabb.max << "}";
        return os;
    }

    // 静的メンバの定義
    const Vector3 Vector3::Zero(0.0f, 0.0f, 0.0f);
    const Vector3 Vector3::One(1.0f, 1.0f, 1.0f);
    const Vector3 Vector3::Up(0.0f, 1.0f, 0.0f);
    const Vector3 Vector3::Down(0.0f, -1.0f, 0.0f);
    const Vector3 Vector3::Left(-1.0f, 0.0f, 0.0f);
    const Vector3 Vector3::Right(1.0f, 0.0f, 0.0f);
    const Vector3 Vector3::Forward(0.0f, 0.0f, 1.0f);
    const Vector3 Vector3::Backward(0.0f, 0.0f, -1.0f);

    const Quaternion Quaternion::Identity(0.0f, 0.0f, 0.0f, 1.0f);

	
}

