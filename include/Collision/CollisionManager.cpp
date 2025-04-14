#include "CollisionManager.h"

#include <algorithm>

namespace Collision{
    Manager::Manager() {
        InitThreadPool();
    }

    Manager::~Manager() {
        // スレッドプールを停止
        running_ = false;
        condition_.notify_all();

        // すべてのスレッドが終了するのを待つ
        for (auto& thread : threadPool_){
            if (thread.joinable()){
                thread.join();
            }
        }
    }

    void Manager::InitThreadPool() {
        for (uint32_t i = 0; i < maxThreadCount_; ++i){
            threadPool_.emplace_back(&Manager::WorkerThread, this);
        }
    }

    void Manager::WorkerThread() {
        while (running_){
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(queueMutex_);
                condition_.wait(lock, [this]{
                    return !taskQueue_.empty() || !running_;
                });

                if (!running_ && taskQueue_.empty()){
                    return;
                }

                if (!taskQueue_.empty()){
                    task = std::move(taskQueue_.front());
                    taskQueue_.pop();
                }
            }

            if (task){
                task();
            }
        }
    }

    void Manager::AddTask(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            taskQueue_.push(std::move(task));
        }
        condition_.notify_one();
    }

    void Manager::WaitForTasks() {
        // すべてのタスクが完了するのを待つ
        std::unique_lock<std::mutex> lock(queueMutex_);
        condition_.wait(lock, [this]{
            return taskQueue_.empty();
        });
    }

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
        std::mutex resultsMutex;

        const size_t chunkSize = std::max(1ULL, count / maxThreadCount_);

        std::atomic<int> tasksCompleted = 0;
        int totalTasks = std::min(maxThreadCount_, static_cast<uint32_t>(count));

        for (uint32_t t = 0; t < totalTasks; ++t){
            const size_t start = t * chunkSize;
            const size_t end = std::min(start + chunkSize, count);
            const uint32_t threadIndex = t;

            AddTask([this, &array, &results, &resultsMutex, start, end, threadIndex, &tasksCompleted](){
	            for (size_t i = start; i < end; ++i){
                    const auto& [id1, c1] = array[i];

                    for (size_t j = i + 1; j < array.size(); ++j){
                        const auto& [id2, c2] = array[j];

                        if (!Filter({id1, id2}))continue;

                        if (Detect(c1, c2)){
                            Pair result = {id1, id2};

                            std::unique_lock lockResult(resultsMutex);
                            if (!result.first.empty() && !result.second.empty()){
                                results[threadIndex] = std::move(result);
                            }
                        }
                    }
                }
                ++tasksCompleted;
            });
        }

        // すべてのタスクが完了するのを待つ（別の方法）
        while (tasksCompleted < totalTasks){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        {
            std::unique_lock lock(mutex_);
            for (const auto& tRes : results){
                if (tRes.first.empty() || tRes.second.empty()) continue;
                detectedPair_.emplace_back(tRes);
            }
        }
    }

    void Manager::ProcessEvent() {
        std::unique_lock lock(mutex_);
        for (const auto& pair : detectedPair_){
            auto itr = colliders_.find(pair.first);
            auto otr = colliders_.find(pair.second);

            if (itr == colliders_.end() || otr == colliders_.end())continue;

            const auto& c1 = itr->second;
            const auto& c2 = otr->second;
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
            auto itr = colliders_.find(pre.first);
            auto otr = colliders_.find(pre.second);

            if (itr == colliders_.end() || otr == colliders_.end())continue;

            const auto& c1 = itr->second;
            const auto& c2 = otr->second;
            if (std::ranges::find(detectedPair_, pre) == detectedPair_.end()){
                c1->OnCollision({EventType::Exit, c2});
                c2->OnCollision({EventType::Exit, c1});
            }
        }
    }

    bool Manager::Filter(const Pair& _pair) const {
        auto itr = colliders_.find(_pair.first);
        auto otr = colliders_.find(_pair.second);

        if (itr == colliders_.end() || otr == colliders_.end())return false;

        const Collider* c1 = itr->second;
        const Collider* c2 = otr->second;
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