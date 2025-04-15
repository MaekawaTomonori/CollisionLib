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
    	using Size = std::variant<float, Vec3>;
		using CBFunc = std::function<void(const Collider*)>;

		std::atomic<bool> enable_ = false;
    	std::atomic<bool> registered_ = false;
		std::shared_mutex mutex_;

		std::string uuid_;

        Type type_ = Type::None;
		Vec3 translate_{};
        Size size_ {};

        Manager* manager_ = nullptr;

		std::array<CBFunc, 3> onCollisions_;

		uint32_t attribute_ = 0b0;
		uint32_t ignore_ = 0b0;

        void* owner_ = nullptr;

	public:
		Collider();
		~Collider();
		void Enable();
		void Disable();

		bool IsEnabled() const;
		bool IsDisabled() const;

		bool IsRegistered() const;

        Collider* SetType(const Type _type);
        Collider* SetTranslate(const Vec3& _translate);
    	Collider* SetSize(const Size _size);
        Collider* SetEvent(EventType _event, std::function<void(const Collider*)> _callback);
        Collider* AddAttribute(uint32_t _attribute);
        Collider* RemoveAttribute(uint32_t _attribute);
        Collider* AddIgnore(uint32_t _ignore);
        Collider* RemoveIgnore(uint32_t _ignore);
        Collider* SetOwner(void* _owner);

        void OnCollision(Event _event) const;

		std::string GetUniqueId() const;
        Type GetType() const;
        uint32_t GetAttribute() const;
        uint32_t GetIgnore() const;
        Size GetSize() const;
        Vec3 GetTranslate() const;
        void* GetOwner() const;
    };
}
