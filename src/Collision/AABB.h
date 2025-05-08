#pragma once
#include "sys/Mathematics.h"

namespace Collision{
    class AABB{
        public:
        Vec3 min; // ç≈è¨ì_
        Vec3 max; // ç≈ëÂì_
        AABB() : min(Vec3::Zero), max(Vec3::Zero) {
        }
        AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {
        }
        bool Intersects(const AABB& other) const {
            return (min.x <= other.max.x && max.x >= other.min.x &&
                    min.y <= other.max.y && max.y >= other.min.y &&
                    min.z <= other.max.z && max.z >= other.min.z);
        }
        bool Contains(const Collision::Vec3& point) const {
            return (point.x >= min.x && point.x <= max.x &&
                    point.y >= min.y && point.y <= max.y &&
                    point.z >= min.z && point.z <= max.z);
        }

        const Vec3& GetMin() const;
        const Vec3& GetMax() const;
    };
}