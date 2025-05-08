#include "Sphere.h"
namespace Collision{
	Sphere::Sphere() {
		transform_ = {.translate= Vec3::Zero, .rotation= Vec3::Zero, .scale= 0.f};
	}
}
