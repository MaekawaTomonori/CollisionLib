#include "Collider.h"

#include <utility>

#include "CollisionManager.h"
#include "sys/Singleton.h"
#include "sys/System.h"

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
        uuid_ = System::CreateUniqueId();
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

	Collider* Collider::SetEvent(EventType _event, std::function<void(const Collider*)> _callback) {
        onCollisions_[static_cast<int>(_event)] = std::move(_callback);
        return this;
	}

	Collider* Collider::AddAttribute(const uint32_t _attribute) {
        attribute_ |= _attribute;
        return this;
	}

	Collider* Collider::RemoveAttribute(const uint32_t _attribute) {
        attribute_ &= ~_attribute;
        return this;
	}

	Collider* Collider::AddIgnore(const uint32_t _ignore) {
        ignore_ |= _ignore;
        return this;
	}

	Collider* Collider::RemoveIgnore(const uint32_t _ignore) {
        ignore_ &= ~_ignore;
        return this;
	}

	void Collider::OnCollision(const Event _event) const {
		if (const CBFunc callback = onCollisions_[static_cast<int>(_event.GetType())]){
            callback(_event.GetOther());
        }
    }

	std::string Collider::GetUniqueId() const {
        return uuid_;
	}

	Type Collider::GetType() const {
        return type_;
	}

    uint32_t Collider::GetAttribute() const {
        return attribute_;
	}

    uint32_t Collider::GetIgnore() const {
        return ignore_;
    }

    Collider::Size Collider::GetSize() const {
        return size_;
    }

    Vector3 Collider::GetTranslate() const {
        return translate_;
    }
}
