#pragma once
#include <array>
#include <atomic>
#include <functional>
#include <shared_mutex>
#include <string>
#include <variant>

#include "ReferencePtr.hpp"
#include "Math/Vector3.hpp"

namespace Collision{
    class Manager;
    class Collider;

    enum class Type{
        Sphere,
        AABB,
        Capsule,
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

    /**
     * 球コライダーの形状。radiusは球の半径。
     */
    struct SphereShape{
        float radius = 0.f;

        SphereShape() = default;
        SphereShape(float _radius) : radius(_radius) {}
    };

    /**
     * AABBコライダーの形状。sizeは各軸の全長(半径ではない)。
     */
    struct AabbShape{
        Vector3 size {};

        AabbShape() = default;
        AabbShape(const Vector3& _size) : size(_size) {}
    };

    /**
     * カプセルコライダーの形状。
     * 始点はコライダーのtranslateとし、終点はtranslate + offsetで表す。
     */
    struct CapsuleShape{
        Vector3 offset {};
        float radius = 0.f;

        CapsuleShape() = default;
        CapsuleShape(const Vector3& _offset, float _radius) : offset(_offset), radius(_radius) {}
    };

    /**
     * UUID・Type・attribute/ignoreフィルタ・ownerといった、
     * ColliderとRay(登録を伴わない)の双方に共通する識別情報だけを扱う基底。
     * 登録・Enable/Disable・イベント通知など「衝突オブジェクトのライフサイクル」に
     * 関わるものはここには含めず、派生クラス側の責務として残す。
     */
    template<class Derived>
    class Identifiable{
    protected:
        Data data_ {};

    public:
        virtual ~Identifiable() = default;

        Derived* SetType(Type _type) {
            data_.type = _type;
            return static_cast<Derived*>(this);
        }

        Derived* AddAttribute(uint32_t _attribute) {
            data_.attribute |= _attribute;
            return static_cast<Derived*>(this);
        }

        Derived* RemoveAttribute(uint32_t _attribute) {
            data_.attribute &= ~_attribute;
            return static_cast<Derived*>(this);
        }

        Derived* AddIgnore(uint32_t _ignore) {
            data_.ignore |= _ignore;
            return static_cast<Derived*>(this);
        }

        Derived* RemoveIgnore(uint32_t _ignore) {
            data_.ignore &= ~_ignore;
            return static_cast<Derived*>(this);
        }

        Derived* SetOwner(void* _owner) {
            data_.owner = _owner;
            return static_cast<Derived*>(this);
        }

        const Data& GetData() const {
            return data_;
        }

        std::string GetUniqueId() const {
            return data_.uuid;
        }

        Type GetType() const {
            return data_.type;
        }

        uint32_t GetAttribute() const {
            return data_.attribute;
        }

        uint32_t GetIgnore() const {
            return data_.ignore;
        }

        void* GetOwner() const {
            return data_.owner;
        }

        bool operator==(const std::string& other) const {
            return data_.uuid == other;
        }
    };

    class Collider : public Identifiable<Collider>{
        using Size = std::variant<SphereShape, AabbShape, CapsuleShape>;
        using CBFunc = std::function<void(const Collider*)>;

        GESTD::ReferencePtr<Manager> manager_ = nullptr;

        std::atomic<bool> enable_ = false;
        std::atomic<bool> registered_ = false;
        std::shared_mutex mutex_;

        Vector3 translate_ {};
        Size size_ {};

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
        Collider* SetTranslate(const Vector3& _translate);
        Collider* SetSize(Size _size);
        Collider* SetEvent(EventType _event, std::function<void(const Collider*)> _callback);

        void OnCollision(Event _event) const;

        std::string GetName() const;
        Size GetSize() const;
        Vector3 GetTranslate() const;
    };

    /**
     * Use in ptr basically
     */
    class Ray : public Identifiable<Ray>{
        Vector3 origin_;
        Vector3 direction_;
        float length_;
        GESTD::ReferencePtr<Manager> manager_ = nullptr;
    public:
        Ray();
        Ray(const Vector3& _origin, const Vector3& _direction, float _length);

        Ray* SetOrigin(const Vector3& _origin);
        Ray* SetDirection(const Vector3& _direction);
        Ray* SetLength(const float& _length);

        Ray* SetDestination(const Vector3& _destination);

        const Vector3& GetOrigin() const;
        const Vector3& GetDirection() const;
        const float& GetLength() const;

        Vector3 GetPoint(float t) const;
    };
}
