#pragma once
#include <array>
#include <functional>
#include <shared_mutex>
#include <variant>

#include "Mathematics.h"

namespace Collision{
	class Manager;
	class Collider;
	class Sphere;
	class AABB;
	class Ray;

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

	 struct Data{
		std::string uuid;
		Type type = Type::None;
		uint32_t attribute = 0b0;
		uint32_t ignore = 0b0;
		void* owner = nullptr;
	};

	class Collider{
		using Size = std::variant<float, Vec3>;
		using CBFunc = std::function<void(const Collider*)>;

		std::atomic<bool> enable_ = false;
		std::atomic<bool> registered_ = false;
		std::shared_mutex mutex_;

		Vec3 translate_ {};
		Size size_ {};

		Data data_ {};

		Manager* manager_ = nullptr;

		std::array<CBFunc, 3> onCollisions_;

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

		const Data& GetData() const;

		std::string GetUniqueId() const;
		Type GetType() const;
		uint32_t GetAttribute() const;
		uint32_t GetIgnore() const;
		Size GetSize() const;
		Vec3 GetTranslate() const;
		void* GetOwner() const;

		bool operator==(const std::string& other) const {
			return data_.uuid == other;
		}
	};

	/// @brief
	/// Use in ptr basically
	/// 
	class Ray{
		Vec3 origin_;
		Vec3 direction_;
		float length_;
		Manager* manager_ = nullptr;
		Data data_ {};
	public:
		Ray();
		Ray(const Vec3& _origin, const Vec3& _direction, float _length);

		Ray* SetOrigin(const Vec3& _origin);
		Ray* SetDirection(const Vec3& _direction);
		Ray* SetLength(const float& _length);

		Ray* SetType(const Type _type);
		Ray* AddAttribute(uint32_t _attribute);
		Ray* RemoveAttribute(uint32_t _attribute);
		Ray* AddIgnore(uint32_t _ignore);
		Ray* RemoveIgnore(uint32_t _ignore);
		Ray* SetOwner(void* _owner);

        Ray* SetDestination(const Vec3& _destination);

        const Data& GetData() const;

		const Vec3& GetOrigin() const;
		const Vec3& GetDirection() const;
		const float& GetLength() const;

		std::string GetUniqueId() const;
		Type GetType() const;
		uint32_t GetAttribute() const;
		uint32_t GetIgnore() const;
		void* GetOwner() const;

		Vec3 GetPoint(float t) const;

		bool operator==(const std::string& other) const;
	};
}
