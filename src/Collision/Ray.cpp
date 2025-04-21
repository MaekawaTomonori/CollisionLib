#include "Ray.h"
#include <cmath>

#include "AABB.h"
#include "sys/Mathematics.h"

namespace Collision {
    Ray::Ray()
        : origin_(Vec3::Zero), direction_(Vec3::Forward), length_(1000.0f) {
    }

    Ray::Ray(const Vec3& origin, const Vec3& direction, float length)
        : origin_(origin), direction_(direction), length_(length) {
        // Normalize direction
        direction_.Normalize();
    }

    const Vec3& Ray::GetOrigin() const {
        return origin_;
    }

    const Vec3& Ray::GetDirection() const {
        return direction_;
    }

    float Ray::GetLength() const {
        return length_;
    }

    bool Ray::IsEnabled() const {
        return enable_;
    }

    void Ray::SetOrigin(const Vec3& origin) {
        origin_ = origin;
    }

    void Ray::SetDirection(const Vec3& direction) {
        direction_ = direction;
        direction_.Normalize();
    }

    void Ray::SetLength(float length) {
        length_ = length;
    }

    void Ray::Enable() {
        enable_ = true;
    }

    void Ray::Disable() {
        enable_ = false;
    }

    Vec3 Ray::GetPointAtDistance(float distance) const {
        return origin_ + direction_ * distance;
    }

    bool Ray::Intersects(const Vec3& point) const {
        Vec3 toPoint = point - origin_;
        float projection = Vec3::Dot(toPoint, direction_);
        if (projection < 0 || projection > length_) return false;

        Vec3 closestPoint = origin_ + direction_ * projection;
        return (closestPoint - point).SquaredLength() < 1e-6f;
    }

    bool Ray::Intersects(const AABB& aabb) const {
        if (!enable_) return false;

        float tmin = 0.0f;
        float tmax = length_;

        // X�������̔���
        if (std::abs(direction_.x) < 1e-6f) {
            // Ray��x���ɕ��s
            if (origin_.x < aabb.min.x || origin_.x > aabb.max.x)
                return false;
        } else {
            // X�������̌������v�Z
            float invD = 1.0f / direction_.x;
            float t1 = (aabb.min.x - origin_.x) * invD;
            float t2 = (aabb.max.x - origin_.x) * invD;

            // �����͈͂𒲐�
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }

        // Y�������̔���
        if (std::abs(direction_.y) < 1e-6f) {
            // Ray��y���ɕ��s
            if (origin_.y < aabb.min.y || origin_.y > aabb.max.y)
                return false;
        } else {
            // Y�������̌������v�Z
            float invD = 1.0f / direction_.y;
            float t1 = (aabb.min.y - origin_.y) * invD;
            float t2 = (aabb.max.y - origin_.y) * invD;

            // �����͈͂𒲐�
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }

        // Z�������̔���
        if (std::abs(direction_.z) < 1e-6f) {
            // Ray��z���ɕ��s
            if (origin_.z < aabb.min.z || origin_.z > aabb.max.z)
                return false;
        } else {
            // Z�������̌������v�Z
            float invD = 1.0f / direction_.z;
            float t1 = (aabb.min.z - origin_.z) * invD;
            float t2 = (aabb.max.z - origin_.z) * invD;

            // �����͈͂𒲐�
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }

        // �����_��Ray�̒������ɂ��邩�m�F
        return tmin <= length_;
    }

    bool Ray::IntersectsSphere(const Vec3& center, float radius) const {
        if (!enable_) return false;

        // ���̒��S����Ray�̎n�_�ւ̃x�N�g��
        Vec3 m = origin_ - center;
        
        // �񎟕������̌W��
        float b = Vec3::Dot(m, direction_);
        float c = Vec3::Dot(m, m) - radius * radius;
        
        // ����Ray�̌��ɂ���A����Ray�����̊O�ɂ���ꍇ�͌������Ȃ�
        if (c > 0.0f && b > 0.0f) return false;
        
        // ���ʎ�
        float discr = b * b - c;
        
        // ���̔��ʎ��͌����Ȃ����Ӗ�����
        if (discr < 0.0f) return false;
        
        // �����_�܂ł̋������v�Z
        float t = -b - std::sqrt(discr);
        
        // �����_�����ɂ���ꍇ�͓�Ԗڂ̌����_���`�F�b�N
        if (t < 0.0f)
            t = -b + std::sqrt(discr);
        
        // �����_��Ray�̌��ɂ���ꍇ�͌����Ȃ�
        if (t < 0.0f) return false;
        
        // �����_��Ray�̒������ɂ��邩�m�F
        return t <= length_;
    }
}
