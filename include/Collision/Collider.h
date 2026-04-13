#pragma once
#include <array>
#include <functional>
#include <shared_mutex>
#include <variant>

#include "ReferencePtr.hpp"
#include "Math/Vector3.hpp"

namespace Collision{
    class Manager;
    class Collider;

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
         std::string name;
         std::string uuid;
         Type type = Type::None;
         uint32_t attribute = 0b0;
         uint32_t ignore = 0b0;
         void* owner = nullptr;
     };

    class Collider{
        using Size = std::variant<float, Vector3>;
        using CBFunc = std::function<void(const Collider*)>;
        
        GESTD::ReferencePtr<Manager> manager_ = nullptr;

        std::atomic<bool> enable_ = false;
        std::atomic<bool> registered_ = false;
        std::shared_mutex mutex_;

        Vector3 translate_ {};
        Size size_ {};
        Data data_ {};

        std::array<CBFunc, 3> onCollisions_;

    public:
        Collider();
        ~Collider();
        void Enable();
        void Disable();

        bool IsEnabled() const;
        bool IsDisabled() const;

        bool IsRegistered() const;

        Collider* SetName(const std::string& _name);
        Collider* SetType(Type _type);
        Collider* SetTranslate(const Vector3& _translate);
        Collider* SetSize(Size _size);
        Collider* SetEvent(EventType _event, std::function<void(const Collider*)> _callback);
        Collider* AddAttribute(uint32_t _attribute);
        Collider* RemoveAttribute(uint32_t _attribute);
        Collider* AddIgnore(uint32_t _ignore);
        Collider* RemoveIgnore(uint32_t _ignore);
        Collider* SetOwner(void* _owner);

        void OnCollision(Event _event) const;

        const Data& GetData() const;

        std::string GetName() const;
        std::string GetUniqueId() const;
        Type GetType() const;
        uint32_t GetAttribute() const;
        uint32_t GetIgnore() const;
        Size GetSize() const;
        Vector3 GetTranslate() const;
        void* GetOwner() const;

        bool operator==(const std::string& other) const {
            return data_.uuid == other;
        }
    };

    /// @brief
    /// Use in ptr basically
    class Ray{
        Vector3 origin_;
        Vector3 direction_;
        float length_;
        GESTD::ReferencePtr<Manager> manager_ = nullptr;
        Data data_ {};
    public:
        Ray();
        Ray(const Vector3& _origin, const Vector3& _direction, float _length);

        Ray* SetOrigin(const Vector3& _origin);
        Ray* SetDirection(const Vector3& _direction);
        Ray* SetLength(const float& _length);

        Ray* SetType(Type _type);
        Ray* AddAttribute(uint32_t _attribute);
        Ray* RemoveAttribute(uint32_t _attribute);
        Ray* AddIgnore(uint32_t _ignore);
        Ray* RemoveIgnore(uint32_t _ignore);
        Ray* SetOwner(void* _owner);

        Ray* SetDestination(const Vector3& _destination);

        const Data& GetData() const;

        const Vector3& GetOrigin() const;
        const Vector3& GetDirection() const;
        const float& GetLength() const;

        std::string GetUniqueId() const;
        Type GetType() const;
        uint32_t GetAttribute() const;
        uint32_t GetIgnore() const;
        void* GetOwner() const;

        Vector3 GetPoint(float t) const;

        bool operator==(const std::string& other) const;
    };
}
