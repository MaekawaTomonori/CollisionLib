#pragma once
#include "sys/Mathematics.h"
#include "AABB.h"

namespace Collision {
    class Ray {
    private:
        Vec3 origin_;
        Vec3 direction_;
        float length_;
        bool enable_ = true;

    public:
        Ray();
        Ray(const Vec3& origin, const Vec3& direction, float length = 1000.0f);

        // Getter methods
        const Vec3& GetOrigin() const;
        const Vec3& GetDirection() const;
        float GetLength() const;
        bool IsEnabled() const;

        // Setter methods
        void SetOrigin(const Vec3& origin);
        void SetDirection(const Vec3& direction);
        void SetLength(float length);
        void Enable();
        void Disable();

        // Intersection methods
        bool Intersects(const AABB& aabb) const;
        bool IntersectsSphere(const Vec3& center, float radius) const;
        Vec3 GetPointAtDistance(float distance) const;
    };
}
