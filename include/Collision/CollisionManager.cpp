#include "CollisionManager.h"

#include <algorithm>

namespace Collision {
    bool Manager::Register(Collider* c) {
        if (!c) return false;

        std::unique_lock lock(mutex_);
        if (c){
            colliders_[c->GetUniqueId()] = c;
            return true;
        }

        return false;
    }

    bool Manager::Unregister(const Collider* c) {
        if (!c)return false;

        std::unique_lock lock(mutex_);
        colliders_.erase(c->GetUniqueId());

        auto it = std::ranges::remove_if(detectedPair_,
                                 [&c](const Pair& pair){
            return pair.first == c->GetUniqueId() || pair.second == c->GetUniqueId();
        });

        return true;
    }

    void Manager::Detect() {
        prePair_.clear();
        prePair_ = std::move(detectedPair_);
        detectedPair_.clear();

        std::vector<std::pair<std::string, Collider*>> array;
        {
            std::shared_lock lock(mutex_);
            for (const auto& [key, value] : colliders_){
                if (value->IsEnabled()){
                    array.emplace_back(key, value);
                }
            }
        }

        const size_t count = array.size();
        std::vector<Pair> results(maxThreadCount_);
        std::vector<std::thread> threads;

        const size_t chunkSize = std::max(1ULL, count / maxThreadCount_);

        for (uint32_t t = 0; t < std::min(maxThreadCount_, static_cast<uint32_t>(count)); ++t){
            const size_t start = t * chunkSize;
            const size_t end = std::min(start + chunkSize, count);
            threads.emplace_back([this, &array, &results, start, end, t](){
                auto& result = results[t];
                for (size_t i = start; i < end; ++i){
                    const auto& [id1, c1] = array[i];

                    for (size_t j = i + 1; j < array.size(); ++j){
                        const auto& [id2, c2] = array[j];

                        if (!Filter({id1, id2}))continue;

                        if (Detect(c1, c2)){
                            result = {id1, id2};
                        }

                    }
                }
            });
        }

        for (auto& thread : threads){
            thread.join();
        }

        {
            std::unique_lock lock(mutex_);
            for (const auto& result : results){
                const auto& [v1, v2] = result;
                detectedPair_.emplace_back(v1, v2);
            }
        }
    }

    void Manager::ProcessEvent() {
        std::unique_lock lock(mutex_);
        for (const auto& pair : detectedPair_){
            const auto& c1 = colliders_.at(pair.first);
            const auto& c2 = colliders_.at(pair.second);
            if (c1 == c2) continue;
            if (std::ranges::find(prePair_, pair) == prePair_.end()){
                c1->OnCollision({EventType::Trigger, c2});
                c2->OnCollision({EventType::Trigger, c1});
            } else{
                c1->OnCollision({EventType::Stay, c2});
                c2->OnCollision({EventType::Stay, c1});
            }
        }

        for (const auto& pre : prePair_){
            const auto& c1 = colliders_.at(pre.first);
            const auto& c2 = colliders_.at(pre.second);
            if (std::ranges::find(detectedPair_, pre) == detectedPair_.end()){
                c1->OnCollision({EventType::Exit, c2});
                c2->OnCollision({EventType::Exit, c1});
            }
        }
    }

    bool Manager::Filter(const Pair& _pair) const {
        const Collider* c1 = colliders_.at(_pair.first);
        const Collider* c2 = colliders_.at(_pair.second);
        if (c1 == c2) return false;
        if (!c1->IsEnabled() || !c2->IsEnabled()) return false;
        if (c1->GetType() == Type::None || c2->GetType() == Type::None) return false;
        if (c1->GetAttribute() & c2->GetIgnore() || c1->GetIgnore() & c2->GetAttribute()) return false;
        return true;
    }
    bool Manager::Detect(const Collider* _c1, const Collider* _c2) {
        bool sp1 = std::holds_alternative<float>(_c1->GetSize());
        bool sp2 = std::holds_alternative<float>(_c2->GetSize());
    	if (sp1 && sp2){
    		// Sphere vs Sphere
            return (_c1->GetTranslate() - _c2->GetTranslate()).Length() <= (std::get<float>(_c1->GetSize()) + std::get<float>(_c2->GetSize()));
        } else if (sp1 && !sp2 || !sp1 && sp2){
            // AABB vs Sphere
            const auto& aabb = sp1 ? _c2 : _c1;
            const auto& sphere = sp1 ? _c1 : _c2;
            const auto& aabbSize = std::get<Vec3>(aabb->GetSize());
            const auto& aabbTranslate = aabb->GetTranslate();
            const auto& sphereSize = std::get<float>(sphere->GetSize());
            const auto& sphereTranslate = sphere->GetTranslate();

            return (abs(aabbTranslate.x - sphereTranslate.x) < aabbSize.x + sphereSize) &&
                (abs(aabbTranslate.y - sphereTranslate.y) < aabbSize.y + sphereSize) &&
                (abs(aabbTranslate.z - sphereTranslate.z) < aabbSize.z + sphereSize);

        } else if (!sp1 && !sp2){
            // AABB vs AABB
            auto size1 = std::get<Vec3>(_c1->GetSize());
            auto size2 = std::get<Vec3>(_c2->GetSize());
            return (std::abs(_c1->GetTranslate().x - _c2->GetTranslate().x) < size1.x + size2.x) &&
                (std::abs(_c1->GetTranslate().y - _c2->GetTranslate().y) < size1.y + size2.y) &&
                (std::abs(_c1->GetTranslate().z - _c2->GetTranslate().z) < size1.z + size2.z);
        }

    	return false;
    }
}
