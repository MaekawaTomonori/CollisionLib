#include "CollisionManager.h"
#include <algorithm>

namespace Collision{
    Manager::Manager() {
        InitThreadPool();
    }

    Manager::~Manager() {
        // スレッドプールを停止
        running_ = false;

        // すべてのスレッドが終了するのを待つ
        for (auto& thread : threadPool_){
            if (thread.joinable()){
                thread.join();
            }
        }
    }

    void Manager::InitThreadPool() {
        for (int i = 0; i < maxThreadCount_; ++i){
            threadPool_.emplace_back(&Manager::WorkerThread, this, i);
        }
    }

    void Manager::WorkerThread(int threadId) const {
        // スレッドプールの各スレッドはここで待機し、
        // Detectメソッドから直接タスクを割り当てられる
        while (running_){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    bool Manager::Register(Collider* collider) {
        if (!collider) return false;

        // 衝突処理中はしばらく待機してからリトライ
        while (isProcessingCollisions_){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::unique_lock lock(mutex_);
        colliders_[collider->GetUniqueId()] = collider;
        return true;
    }

    bool Manager::Unregister(const Collider* collider) {
        if (!collider) return false;

        // 衝突処理中はしばらく待機してからリトライ
        while (isProcessingCollisions_){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::unique_lock lock(mutex_);
        colliders_.erase(collider->GetUniqueId());

        auto it = std::ranges::remove_if(detectedPair_,
                                         [&collider](const Pair& pair){
            return pair.first == collider->GetUniqueId() || pair.second == collider->GetUniqueId();
        });

        return true;
    }

    void Manager::Detect() {
        // 衝突処理中フラグを立てる
        isProcessingCollisions_ = true;

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
        if (count == 0){
            isProcessingCollisions_ = false;
            return;
        }

        // 結果格納用の配列
        std::vector<std::vector<Pair>> threadResults(maxThreadCount_);

        // タスク分割
        const size_t chunkSize = std::max(1ULL, count / maxThreadCount_);

        // 完了カウンター
        std::atomic<int> completedThreads = 0;

        // 各スレッドに仕事を割り当て
        for (uint32_t t = 0; t < std::min(maxThreadCount_, static_cast<uint32_t>(count)); ++t){
            const uint32_t threadIndex = t;
            const size_t start = t * chunkSize;
            const size_t end = std::min(start + chunkSize, count);

            // スレッドプールを使わずに直接スレッドを生成（修正ポイント）
            std::thread worker([this, &array, &threadResults, &completedThreads, start, end, threadIndex](){
                std::vector<Pair> localResults;

                for (size_t i = start; i < end; ++i){
                    const auto& [id1, c1] = array[i];

                    for (size_t j = i + 1; j < array.size(); ++j){
                        const auto& [id2, c2] = array[j];

                        if (!Filter({id1, id2})) continue;

                        if (Detect(c1, c2)){
                            localResults.emplace_back(id1, id2);
                        }
                    }
                }

                // ローカル結果をスレッド結果配列に保存
                threadResults[threadIndex] = std::move(localResults);
                ++completedThreads;
            });

            // 即座にデタッチして管理から外す
            worker.detach();
        }

        // すべてのスレッドが完了するのを待つ
        while (completedThreads < std::min(maxThreadCount_, static_cast<uint32_t>(count))){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // 結果をマージ
        {
            std::unique_lock lock(mutex_);
            for (const auto& results : threadResults){
                for (const auto& pair : results){
                    detectedPair_.push_back(pair);
                }
            }
        }

        // 衝突処理完了フラグを下げる
        isProcessingCollisions_ = false;
    }

    void Manager::ProcessEvent() {
        // 衝突処理中フラグを立てる
        isProcessingCollisions_ = true;

        //std::unique_lock lock(mutex_);
        for (const auto& pair : detectedPair_){
            auto itr = colliders_.find(pair.first);
            auto otr = colliders_.find(pair.second);

            if (itr == colliders_.end() || otr == colliders_.end()) continue;

            const auto& c1 = itr->second;
            const auto& c2 = otr->second;
            if (c1 == c2) continue;

            bool foundInPre = false;
            for (const auto& pre : prePair_){
                if ((pre.first == pair.first && pre.second == pair.second) ||
                    (pre.first == pair.second && pre.second == pair.first)){
                    foundInPre = true;
                    break;
                }
            }

            if (!foundInPre){
                // ロックを一時的に解放してコールバック呼び出し
                //lock.unlock();
                c1->OnCollision({EventType::Trigger, c2});
                c2->OnCollision({EventType::Trigger, c1});
                //lock.lock();
            } else{
                // ロックを一時的に解放してコールバック呼び出し
                //lock.unlock();
                c1->OnCollision({EventType::Stay, c2});
                c2->OnCollision({EventType::Stay, c1});
                //lock.lock();
            }
        }

        for (const auto& pre : prePair_){
            auto itr = colliders_.find(pre.first);
            auto otr = colliders_.find(pre.second);

            if (itr == colliders_.end() || otr == colliders_.end()) continue;

            const auto& c1 = itr->second;
            const auto& c2 = otr->second;

            bool foundInDetected = false;
            for (const auto& detected : detectedPair_){
                if ((detected.first == pre.first && detected.second == pre.second) ||
                    (detected.first == pre.second && detected.second == pre.first)){
                    foundInDetected = true;
                    break;
                }
            }

            if (!foundInDetected){
                // ロックを一時的に解放してコールバック呼び出し
                //lock.unlock();
                c1->OnCollision({EventType::Exit, c2});
                c2->OnCollision({EventType::Exit, c1});
                //lock.lock();
            }
        }

        // 衝突処理完了フラグを下げる
        isProcessingCollisions_ = false;
    }

    bool Manager::Filter(const Pair& pair) const {
        auto itr = colliders_.find(pair.first);
        auto otr = colliders_.find(pair.second);

        if (itr == colliders_.end() || otr == colliders_.end()) return false;

        const Collider* c1 = itr->second;
        const Collider* c2 = otr->second;
        if (c1 == c2) return false;
        if (!c1->IsEnabled() || !c2->IsEnabled()) return false;
        if (c1->GetType() == Type::None || c2->GetType() == Type::None) return false;
        if (c1->GetAttribute() & c2->GetIgnore() || c1->GetIgnore() & c2->GetAttribute()) return false;
        return true;
    }

    bool Manager::Detect(const Collider* c1, const Collider* c2) {
        bool sp1 = std::holds_alternative<float>(c1->GetSize());
        bool sp2 = std::holds_alternative<float>(c2->GetSize());
        if (sp1 && sp2){
            // Sphere vs Sphere
            return (c1->GetTranslate() - c2->GetTranslate()).Length() <= (std::get<float>(c1->GetSize()) + std::get<float>(c2->GetSize()));
        } else if (sp1 && !sp2 || !sp1 && sp2){
            // AABB vs Sphere
            const auto& aabb = sp1 ? c2 : c1;
            const auto& sphere = sp1 ? c1 : c2;
            const auto& aabbSize = std::get<Vec3>(aabb->GetSize());
            const auto& aabbTranslate = aabb->GetTranslate();
            const auto& sphereSize = std::get<float>(sphere->GetSize());
            const auto& sphereTranslate = sphere->GetTranslate();

            return (abs(aabbTranslate.x - sphereTranslate.x) < aabbSize.x + sphereSize) &&
                (abs(aabbTranslate.y - sphereTranslate.y) < aabbSize.y + sphereSize) &&
                (abs(aabbTranslate.z - sphereTranslate.z) < aabbSize.z + sphereSize);

        } else if (!sp1 && !sp2){
            // AABB vs AABB
            auto size1 = std::get<Vec3>(c1->GetSize());
            auto size2 = std::get<Vec3>(c2->GetSize());
            return (std::abs(c1->GetTranslate().x - c2->GetTranslate().x) < size1.x + size2.x) &&
                (std::abs(c1->GetTranslate().y - c2->GetTranslate().y) < size1.y + size2.y) &&
                (std::abs(c1->GetTranslate().z - c2->GetTranslate().z) < size1.z + size2.z);
        }

        return false;
    }
}