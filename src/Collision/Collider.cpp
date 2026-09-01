#include "Collision/Collider.h"

#include <format>
#include <utility>

#include "Collision/CollisionManager.h"
#include "Pattern/Singleton.hpp"
#include "Utils.hpp"

namespace Collision{
    Event::Event(EventType _type, const Collider* _collider) :type_(_type), other_(_collider) {
    }

    EventType Event::GetType() const {
        return type_;
    }

    const Collider* Event::GetOther() const {
        return other_;
    }

    Collider::Collider() :manager_(Singleton<Manager>::GetInstance()){
        data_.uuid = Utils::GenerateUniqueId();
        if (!manager_->Register(this)){
            throw std::runtime_error("Failed to register collider");
        }
        registered_ = true;
    }

    Collider::~Collider() {
        if (!manager_->Unregister(this)){
            //WARNING
        }
    }

    void Collider::Enable() {
        enable_ = true;
    }

    void Collider::Disable() {
        enable_ = false;
    }

    bool Collider::IsEnabled() const {
        return enable_;
    }

    bool Collider::IsDisabled() const {
        return !enable_;
    }

    bool Collider::IsRegistered() const{
        return registered_;
    }

    Collider* Collider::SetName(const std::string& _name) {
        data_.name = _name;
        return this;
    }

    Collider* Collider::SetTranslate(const Vector3& _translate) {
        translate_ = _translate;
        return this;
    }

    Collider* Collider::SetSize(const Size _size) {
        size_ = _size;
        return this;
    }

    Collider* Collider::SetEvent(EventType _event, std::function<void(const Collider*)> _callback) {
        onCollisions_[static_cast<int>(_event)] = std::move(_callback);
        return this;
    }

    void Collider::OnCollision(const Event _event) const {
        if (const CBFunc callback = onCollisions_[static_cast<int>(_event.GetType())]){
            callback(_event.GetOther());
        }
    }

    std::string Collider::GetName() const {
        return data_.name;
    }

    Collider::Size Collider::GetSize() const {
        return size_;
    }

    Vector3 Collider::GetTranslate() const {
        return translate_;
    }

    Ray::Ray() :origin_({}), direction_({}), length_(0), manager_(Singleton<Manager>::GetInstance()) {
        data_.uuid = Utils::GenerateUniqueId();
        data_.type = Type::Ray;
    }

    Ray::Ray(const Vector3& origin, const Vector3& direction, float length) :Ray(){
        origin_ = origin;
        direction_ = direction.Normalize();
        length_ = length;
    }


    Ray* Ray::SetOrigin(const Vector3& _origin) {
        origin_ = _origin;
        return this;
    }

    Ray* Ray::SetDirection(const Vector3& _direction) {
        direction_ = _direction.Normalize();
        return this;
    }

    Ray* Ray::SetLength(const float& _length) {
        length_ = _length;
        return this;
    }

    Ray* Ray::SetDestination(const Vector3& _destination) {
        direction_ = (_destination - origin_).Normalize();
        return this;
    }

    const Vector3& Ray::GetOrigin() const {
        return origin_;
    }

    const Vector3& Ray::GetDirection() const {
        return direction_;
    }

    const float& Ray::GetLength() const {
        return length_;
    }

    Vector3 Ray::GetPoint(float t) const {
        return origin_ + direction_ * t;
    }
}
