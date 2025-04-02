#pragma once
#include <shared_mutex>
#include <unordered_set>

#include "Collider.h"

namespace Collision{
    class Manager{
        std::unordered_set<Collider*> colliders_;
        std::shared_mutex mutex_;

    public:
	    void Register(Collider* c);
        void Unregister(Collider* c);
    };
}

