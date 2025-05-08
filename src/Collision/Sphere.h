#pragma once
#include "Collision/Collider.h"
#include "sys/Mathematics.h"

namespace Collision{
	class Sphere : public CollideBody{
    public:
	    Sphere();
	    Sphere(const Vec3& center, float radius);
	    void SetCenter(const Vec3& center);
	    void SetRadius(float radius);
	    const Vec3& GetCenter() const;
	    float GetRadius() const;
	};
}
