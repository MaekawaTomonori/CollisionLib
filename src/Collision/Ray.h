#pragma once
#include "sys/Mathematics.h"

namespace Collision {
    class Sphere;
    class AABB;

    /**
     * Ray representation for collision detection.
     * Represents a ray with origin point and direction vector.
     */
    class Ray{
        public:
        /**
         * Default constructor.
         */
        Ray();

        /**
         * Constructor with origin and direction.
         * @param origin The origin point of the ray.
         * @param direction The direction of the ray (will be normalized).
         */
        Ray(const Vec3& origin, const Vec3& direction);

        /**
         * Sets the origin point of the ray.
         * @param origin The new origin point.
         */
        void SetOrigin(const Vec3& origin);

        /**
         * Sets the direction of the ray.
         * @param direction The new direction (will be normalized).
         */
        void SetDirection(const Vec3& direction);

        /**
         * Gets the origin point of the ray.
         * @return The origin point.
         */
        const Vec3& GetOrigin() const;

        /**
         * Gets the direction vector of the ray.
         * @return The normalized direction vector.
         */
        const Vec3& GetDirection() const;

        /**
         * Gets point at distance along ray.
         * @param distance The distance along the ray.
         * @return The point at the specified distance.
         */
        Vec3 GetPoint(float distance) const;

        /**
         * Checks if this ray intersects with a sphere.
         * @param sphere The sphere to check against.
         * @param distance Output parameter for the distance to intersection.
         * @return True if intersection occurs, false otherwise.
         */
        bool Intersect(const Sphere& sphere, float* distance = nullptr) const;

        /**
         * Checks if this ray intersects with an AABB.
         * @param aabb The AABB to check against.
         * @param distance Output parameter for the distance to intersection.
         * @return True if intersection occurs, false otherwise.
         */
        bool Intersect(const AABB& aabb, float* distance = nullptr) const;

    private:
        Vec3 origin_;      // Ray origin point
        Vec3 direction_;   // Normalized direction vector
    };
}
