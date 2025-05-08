#include "Collision/Collider.h"

#include <format>
#include <utility>

#include "Collision/CollisionManager.h"
#include "src/sys/Singleton.h"
#include "src/sys/System.h"

namespace Collision{
	Event::Event(EventType _type, const Collider* _collider) :type_(_type), other_(_collider) {
	}

	EventType Event::GetType() const {
        return type_;
	}

	const Collider* Event::GetOther() const {
        return other_;
	}

	Collider::Collider() :manager_(Singleton<Manager>::Get()){
        data_.uuid = System::CreateUniqueId();
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

    Collider* Collider::SetType(const Type _type) {
        data_.type = _type;
        return this;
    }

    Collider* Collider::SetTranslate(const Vec3& _translate) {
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

    Vec3 Collider::GetTranslate() const {
        return translate_;
    }

    void* Collider::GetOwner() const {
        return data_.owner;
    }

    Ray::Ray() :origin_({}), direction_({}), length_(0), manager_(Singleton<Manager>::Get()) {
        data_.uuid = System::CreateUniqueId();
        data_.type = Type::Ray;
    }

    Ray::Ray(const Vec3& origin, const Vec3& direction, float length) :Ray(){
        origin_ = origin;
        direction_ = direction;
        direction_.Normalize();
        length_ = length;
    }

    const Vec3& Ray::GetOrigin() const {
        return origin_;
    }

    const Vec3& Ray::GetDirection() const {
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

    Vec3 Ray::GetPoint(float t) const {
        return origin_ + direction_ * t;
    }
}
