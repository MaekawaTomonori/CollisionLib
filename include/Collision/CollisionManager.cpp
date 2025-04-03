#include "CollisionManager.h"

namespace Collision {
    void Manager::Register(Collider* c) {
        std::unique_lock lock(mutex_);
        colliders_.insert(c);
    }

    void Manager::Unregister(Collider* c) {
        std::unique_lock lock(mutex_);
        colliders_.erase(c);
    }

}
