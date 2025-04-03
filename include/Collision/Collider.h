#pragma once
#include <functional>
#include <shared_mutex>
#include <variant>

#include "sys/Mathematics.h"

namespace Collision {
	class Manager;
	class Collider;

	enum class Type{
		AABB,
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
		Event(EventType, const Collider*);
		EventType GetType() const;
        const Collider* GetOther() const;
	};

    class Collider{
    	using Size = std::variant<float, Vector3>;
		using CBFunc = std::function<void(const Collider*)>;

		std::atomic<bool> enable_ = false;
    	std::atomic<bool> registered_ = false;
		std::shared_mutex mutex_;

        Type type_ = Type::None;
		Vector3 translate_{};
        Size size_ {};

        Manager* manager_ = nullptr;

		std::array<CBFunc, 3> onCollisions_;

	public:
		Collider();
		void Enable();
		void Disable();

		bool IsEnabled() const;
		bool IsDisabled() const;

		bool IsRegistered() const;

		Collider* Register();
        Collider* SetType(const Type _type);
        Collider* SetTranslate(const Vector3& _translate);
    	Collider* SetSize(const Size _size);
        Collider* SetOnCollision(EventType _event, std::function<void(const Collider*)> _callback);

        void OnCollision(Event _event) const;
    };
}
