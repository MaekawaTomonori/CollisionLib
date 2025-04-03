#include "Collider.h"

#include <utility>

#include "CollisionManager.h"
#include "sys/Singleton.h"

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

    Collider* Collider::Register() {
        manager_->Register(this);
        registered_ = true;
        return this;
    }

    Collider* Collider::SetType(const Type _type) {
        type_ = _type;
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

	Collider* Collider::SetOnCollision(EventType _event, std::function<void(const Collider*)> _callback) {
        onCollisions_[static_cast<int>(_event)] = std::move(_callback);
        return this;
	}

	void Collider::OnCollision(const Event _event) const {
		if (const CBFunc callback = onCollisions_[static_cast<int>(_event.GetType())]){
            callback(_event.GetOther());
        }
    }
}
