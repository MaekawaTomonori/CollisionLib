#include "Ray.h"

#include "AABB.h"
#include "Sphere.h"

namespace Collision{
	Ray::Ray() : origin_(0.0f, 0.0f, 0.0f), direction_(0.0f, 0.0f, 1.0f) {
    }

	Ray::Ray(const Vec3& origin, const Vec3& direction)
        : origin_(origin) {
        SetDirection(direction);  // 方向ベクトルを正規化
    }

    void Ray::SetOrigin(const Vec3& origin) {
        origin_ = origin;
    }

    void Ray::SetDirection(const Vec3& direction) {
        // 方向ベクトルを必ず正規化する
        direction_ = direction;
        direction_.Normalize();
    }

    const Vec3& Ray::GetOrigin() const {
        return origin_;
    }

    const Vec3& Ray::GetDirection() const {
        return direction_;
    }

    Vec3 Ray::GetPoint(float distance) const {
        return origin_ + direction_ * distance;
    }

    bool Ray::Intersect(const Sphere& sphere, float* distance) const {
        // 原点-球の中心へのベクトル
        Vec3 m = origin_ - sphere.GetCenter();

        // 係数計算（オーバーヘッド削減のため事前計算）
        float b = Vec3::Dot(m, direction_);
        float c = Vec3::Dot(m, m) - sphere.GetRadius() * sphere.GetRadius();

        // 球の外側にいて、球から遠ざかる場合は早期リターン
        if (c > 0.0f && b > 0.0f){
            return false;
        }

        float discriminant = b * b - c;

        // 判別式が負なら交差なし
        if (discriminant < 0.0f){
            return false;
        }

        // 交差距離の計算
        float t = -b - std::sqrt(discriminant);

        // 交差点が始点より手前なら、始点の反対側に交差
        if (t < 0.0f){
            t = -b + std::sqrt(discriminant);
        }

        // それでも負なら交差なし（レイの後ろ側）
        if (t < 0.0f){
            return false;
        }

        if (distance){
            *distance = t;
        }

        return true;
    }

    bool Ray::Intersect(const AABB& aabb, float* distance) const {
        Vec3 min = aabb.GetMin();
        Vec3 max = aabb.GetMax();
        float tmin = 0.0f;
        float tmax = std::numeric_limits<float>::max();

        // 各軸で交差判定（スラブ法）
        for (int i = 0; i < 3; ++i){
            if (std::abs(direction_[i]) < std::numeric_limits<float>::epsilon()){
                // レイの方向がこの軸と平行のとき
                if (origin_[i] < min[i] || origin_[i] > max[i]){
                    return false;
                }
            } else{
                float ood = 1.0f / direction_[i];
                float t1 = (min[i] - origin_[i]) * ood;
                float t2 = (max[i] - origin_[i]) * ood;

                if (t1 > t2){
                    std::swap(t1, t2);
                }

                tmin = t1 > tmin ? t1 : tmin;
                tmax = t2 < tmax ? t2 : tmax;

                if (tmin > tmax){
                    return false;
                }
            }
        }

        if (distance){
            *distance = tmin;
        }

        return true;
    }

}  // namespace CollisionLib