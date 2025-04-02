#pragma once
#include <functional>
#include <shared_mutex>
#include <variant>

#include "Math.h"

namespace Collision{
	class Collider;

	enum class Type{
		Box,
		Sphere,

		None
	};

	enum class EventType{
		Trigger,
		Stay,
        Exit
	};

	class Event{
        EventType type_;
        const Collider* other_;

	public:
		Event(Type, Collider*);
		EventType GetType() const;
		Collider* GetOther() const;
	};

    class Collider{
    	using Size = std::variant<float, Vector3>;
		using CBFunc = std::function<void(const Collider*)>;

    	std::atomic<bool> registered_ = false;
		std::shared_mutex mutex_;

        Type type_ = Type::None;
		bool enable_ = true;
		Vector3 translate_{};
        Size size_ {};

		std::array<CBFunc, 3> onCollisions_;

	public:
		void Enable();
		void Disable();

		bool IsEnabled() const;
		bool IsDisabled() const;

		bool IsRegistered() const;

        Collider* SetType(Type _type);
        Collider* SetTranslate(const Vector3& _translate);
    	Collider* SetSize(Size _size);
        Collider* SetOnCollision(EventType _event, std::function<void(const Collider*)> _callback);

        void OnCollision(Event _event) const;
    };
}
