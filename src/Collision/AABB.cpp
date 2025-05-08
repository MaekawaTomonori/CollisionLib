#include "AABB.h"
namespace Collision {
	const Vec3& AABB::GetMin() const {
		return min;
	}

	const Vec3& AABB::GetMax() const {
        return max;
	}
}
