#include "Mathematics.h"

namespace Collision {
    // 静的メンバの定義
    const Vec3 Vec3::Zero = Vec3(0.0f, 0.0f, 0.0f);
    const Vec3 Vec3::One = Vec3(1.0f, 1.0f, 1.0f);
    const Vec3 Vec3::Up = Vec3(0.0f, 1.0f, 0.0f);
    const Vec3 Vec3::Down = Vec3(0.0f, -1.0f, 0.0f);
    const Vec3 Vec3::Left = Vec3(-1.0f, 0.0f, 0.0f);
    const Vec3 Vec3::Right = Vec3(1.0f, 0.0f, 0.0f);
    const Vec3 Vec3::Forward = Vec3(0.0f, 0.0f, 1.0f);
    const Vec3 Vec3::Backward = Vec3(0.0f, 0.0f, -1.0f);

    Vec3::Vec3(): x(0.0f), y(0.0f), z(0.0f) {
    }

    Vec3::Vec3(float x, float y, float z): x(x), y(y), z(z) {
    }

    Vec3 Vec3::operator+(const Vec3& other) const {
	    return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 Vec3::operator-(const Vec3& other) const {
	    return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 Vec3::operator-(const float other) const {
	    return Vec3(x - other, y - other, z - other);
    }

    Vec3 Vec3::operator*(float scalar) const {
	    return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 Vec3::operator/(float scalar) const {
	    return Vec3(x / scalar, y / scalar, z / scalar);
    }

    Vec3& Vec3::operator+=(const Vec3& other) {
	    x += other.x;
	    y += other.y;
	    z += other.z;
	    return *this;
    }

    Vec3& Vec3::operator-=(const Vec3& other) {
	    x -= other.x;
	    y -= other.y;
	    z -= other.z;
	    return *this;
    }

    Vec3& Vec3::operator*=(float scalar) {
	    x *= scalar;
	    y *= scalar;
	    z *= scalar;
	    return *this;
    }

    Vec3& Vec3::operator/=(float scalar) {
	    x /= scalar;
	    y /= scalar;
	    z /= scalar;
	    return *this;
    }

    bool Vec3::operator==(const Vec3& other) const {
	    return x == other.x && y == other.y && z == other.z;
    }

    bool Vec3::operator!=(const Vec3& other) const {
	    return !(*this == other);
    }

    float Vec3::Length() const {
	    return std::sqrt(x * x + y * y + z * z);
    }

    float Vec3::SquaredLength() const {
	    return x * x + y * y + z * z;
    }

    Vec3 Vec3::Normalized() const {
	    float len = Length();
	    if (len < 0.0001f){
		    return Vec3(0, 0, 0);
	    }
	    return *this / len;
    }

    void Vec3::Normalize() {
	    float len = Length();
	    if (len >= 0.0001f){
		    *this /= len;
	    }
    }

    float Vec3::Dot(const Vec3& a, const Vec3& b) {
	    return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Vec3 Vec3::Cross(const Vec3& a, const Vec3& b) {
	    return Vec3(
		    a.y * b.z - a.z * b.y,
		    a.z * b.x - a.x * b.z,
		    a.x * b.y - a.y * b.x
	    );
    }

    Vec3 Vec3::Lerp(const Vec3& a, const Vec3& b, float t) {
	    return a + (b - a) * t;
    }

    Vec3i::Vec3i(): x(0), y(0), z(0) {
    }

    Vec3i::Vec3i(int x, int y, int z): x(x), y(y), z(z) {
    }

    bool Vec3i::operator==(const Vec3i& other) const {
	    return x == other.x && y == other.y && z == other.z;
    }

    bool Vec3i::operator!=(const Vec3i& other) const {
	    return !(*this == other);
    }

    AABB::AABB() {
	    min = {0,0,0};
	    max = {0,0,0};
    }

    AABB::AABB(const Vec3& mi, const Vec3& ma) {
	    min = mi;
	    max = ma;
    }

    Vec3 AABB::GetCenter() const {
	    return (min + max) * 0.5f;
    }

    Vec3 AABB::GetExtents() const {
	    return (max - min) * 0.5f;
    }

    Vec3 AABB::GetSize() const {
	    return max - min;
    }

    float AABB::GetVolume() const {
	    Vec3 size = GetSize();
	    return size.x * size.y * size.z;
    }

    bool AABB::Contains(const Vec3& point) const {
	    return point.x >= min.x && point.x <= max.x &&
		    point.y >= min.y && point.y <= max.y &&
		    point.z >= min.z && point.z <= max.z;
    }

    bool AABB::Intersects(const AABB& other) const {
	    return min.x <= other.max.x && max.x >= other.min.x &&
		    min.y <= other.max.y && max.y >= other.min.y &&
		    min.z <= other.max.z && max.z >= other.min.z;
    }

    void AABB::Expand(const Vec3& point) {
	    min.x = std::min(min.x, point.x);
	    min.y = std::min(min.y, point.y);
	    min.z = std::min(min.z, point.z);

	    max.x = std::max(max.x, point.x);
	    max.y = std::max(max.y, point.y);
	    max.z = std::max(max.z, point.z);
    }

    void AABB::Expand(const AABB& other) {
	    min.x = std::min(min.x, other.min.x);
	    min.y = std::min(min.y, other.min.y);
	    min.z = std::min(min.z, other.min.z);

	    max.x = std::max(max.x, other.max.x);
	    max.y = std::max(max.y, other.max.y);
	    max.z = std::max(max.z, other.max.z);
    }

    AABB AABB::FromCenterExtents(const Vec3& center, const Vec3& extents) {
	    return AABB(center - extents, center + extents);
    }

    Ray::Ray(): origin(0.0f, 0.0f, 0.0f), direction(0.0f, 0.0f, 1.0f) {
    }

    Ray::Ray(const Vec3& origin, const Vec3& direction): origin(origin), direction(direction.Normalized()) {
    }

    Vec3 Ray::GetPoint(float distance) const {
	    return origin + direction * distance;
    }

    bool Ray::Intersects(const AABB& aabb, float& distance) const {
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
}
