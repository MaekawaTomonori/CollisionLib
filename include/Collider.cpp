#include "Collider.h"

#include <utility>

namespace Collision{
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

    Collider* Collider::SetType(Type _type) {
        type_ = _type;
        return this;
    }

    Collider* Collider::SetTranslate(const Vector3& _translate) {
        translate_ = _translate;
        return this;
    }

    Collider* Collider::SetSize(Size _size) {
        size_ = _size;
        return this;
    }

	Collider* Collider::SetOnCollision(EventType _event, std::function<void(const Collider*)> _callback) {
        onCollisions_[static_cast<int>(_event)] = std::move(_callback);
        return this;
	}

	void Collider::OnCollision(Event _event) const {
		if (const CBFunc callback = onCollisions_[static_cast<int>(_event.GetType())]){
            callback(_event.GetOther());
        }
    }
}