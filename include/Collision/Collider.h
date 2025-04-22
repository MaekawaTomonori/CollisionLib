#pragma once
#include <array>
#include <functional>
#include <shared_mutex>
#include <variant>

#include "sys/Mathematics.h"

namespace Collision {
	class Manager;
	class Collider;
	class Sphere;
	class AABB;
	class Ray;

	class CollideBody{
        Vec3 translate_;
        std::variant<float, Vec3> size_;
		
	};

	enum class Type{
		Sphere,
		AABB,
        Ray,

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
        using Body = std::variant<Sphere, AABB, Ray>;
    	using Size = std::variant<float, Vec3>;
		using CBFunc = std::function<void(const Collider*)>;

		std::atomic<bool> enable_ = false;
    	std::atomic<bool> registered_ = false;
		std::shared_mutex mutex_;

		std::string uuid_;

		//Set
		Type type_ = Type::None;
		std::unique_ptr<Body> body_ = nullptr;

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
        const Body& GetBody() const;
        Type GetType() const;
        uint32_t GetAttribute() const;
        uint32_t GetIgnore() const;
        Vec3 GetTranslate() const;
        Size GetSize() const;
        void* GetOwner() const;
    };
}
