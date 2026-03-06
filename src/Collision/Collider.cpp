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

    Collider* Collider::SetType(const Type _type) {
        data_.type = _type;
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

    Collider* Collider::AddAttribute(const uint32_t _attribute) {
        data_.attribute |= _attribute;
        return this;
    }

    Collider* Collider::RemoveAttribute(const uint32_t _attribute) {
        data_.attribute &= ~_attribute;
        return this;
    }

    Collider* Collider::AddIgnore(const uint32_t _ignore) {
        data_.ignore |= _ignore;
        return this;
    }

    Collider* Collider::RemoveIgnore(const uint32_t _ignore) {
        data_.ignore &= ~_ignore;
        return this;
    }

    Collider* Collider::SetOwner(void* _owner) {
        data_.owner = _owner;
        return this;
    }

    void Collider::OnCollision(const Event _event) const {
        if (const CBFunc callback = onCollisions_[static_cast<int>(_event.GetType())]){
            callback(_event.GetOther());
        }
    }

    const Data& Collider::GetData() const {
        return data_;
    }

    std::string Collider::GetName() const {
        return data_.name;
    }

    std::string Collider::GetUniqueId() const {
        return data_.uuid;
    }

    Type Collider::GetType() const {
        return data_.type;
    }

    uint32_t Collider::GetAttribute() const {
        return data_.attribute;
    }

    uint32_t Collider::GetIgnore() const {
        return data_.ignore;
    }

    Collider::Size Collider::GetSize() const {
        return size_;
    }

    Vector3 Collider::GetTranslate() const {
        return translate_;
    }

    void* Collider::GetOwner() const {
        return data_.owner;
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

    Ray* Ray::SetType(const Type _type) {
        data_.type = _type;
        return this;
    }

    Ray* Ray::AddAttribute(uint32_t _attribute) {
        data_.attribute |= _attribute;
        return this;
    }

    Ray* Ray::RemoveAttribute(uint32_t _attribute) {
        data_.attribute &= ~_attribute;
        return this;
    }

    Ray* Ray::AddIgnore(uint32_t _ignore) {
        data_.ignore |= _ignore;
        return this;
    }

    Ray* Ray::RemoveIgnore(uint32_t _ignore) {
        data_.ignore &= ~_ignore;
        return this;
    }

    Ray* Ray::SetOwner(void* _owner) {
        data_.owner = _owner;
        return this;
    }

    Ray* Ray::SetDestination(const Vector3& _destination) {
        direction_ = (_destination - origin_).Normalize();
        return this;
    }


    const Data& Ray::GetData() const {
        return data_;
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

    std::string Ray::GetUniqueId() const {
        return data_.uuid;
    }

    Type Ray::GetType() const {
        return data_.type;
    }

    uint32_t Ray::GetAttribute() const {
        return data_.attribute;
    }

    uint32_t Ray::GetIgnore() const {
        return data_.ignore;
    }

    void* Ray::GetOwner() const {
        return data_.owner;
    }

    Vector3 Ray::GetPoint(float t) const {
        return origin_ + direction_ * t;
    }

    bool Ray::operator==(const std::string& other) const {
        return data_.uuid == other;
    }
}
