#include "Collision/CollisionManager.h"
#include <algorithm>
#include <condition_variable>
#include <queue>
#include <functional>
#include <ranges>

#include <EventTimer/EventTimer.h>

namespace Collision{
    Manager::Manager() {
        InitThreadPool();
    }

    Manager::~Manager() {
        // スレッドプールを停止
        running_ = false;
        taskCondition_.notify_all();

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
                std::unique_lock<std::mutex> lock(taskMutex_);

                // タスクがあるか終了するまで待機
                taskCondition_.wait(lock, [this]{
                    return !tasks_.empty() || !running_;
                });

                // 終了条件
                if (!running_ && tasks_.empty()){
                    return;
                }

                // タスクを取得
                if (!tasks_.empty()){
                    task = std::move(tasks_.front());
                    tasks_.pop();
                }
            }

            // タスクを実行
            if (task){
                task();
            }
        }
    }

    void Manager::AddTask(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(taskMutex_);
            tasks_.push(std::move(task));
        }
        taskCondition_.notify_one();
    }

    void Manager::WaitForTasks() {
        std::unique_lock<std::mutex> lock(taskMutex_);
        taskCondition_.wait(lock, [this]{
            return tasks_.empty();
        });
    }

    bool Manager::Register(Collider* c) {
        if (!c) return false;

        // 衝突処理中なら遅延登録
        if (isProcessingCollisions_){
            std::unique_lock<std::mutex> lock(pendingMutex_);
            pendingQueue_.push(c);
            return true;
        }

        // 通常登録
        std::unique_lock lock(mutex_);
        colliders_[c->GetUniqueId()] = c;
        return true;
    }

    bool Manager::Unregister(const Collider* c) {
        if (!c) return false;

        // 衝突処理中なら遅延解除
        if (isProcessingCollisions_){
            std::unique_lock<std::mutex> lock(pendingMutex_);
            unregisterQueue_.push(c);
            return true;
        }

        // 通常解除
        std::unique_lock lock(mutex_);
        colliders_.erase(c->GetUniqueId());

        auto it = std::ranges::remove_if(detectedPair_,
                                         [&c](const Pair& pair){
            return pair.first == c->GetUniqueId() || pair.second == c->GetUniqueId();
        });

        return true;
    }

    void Manager::ProcessPendingRegistrations() {
        std::queue<Collider*> registrations;
        std::queue<const Collider*> unregistrations;

        {
            std::unique_lock<std::mutex> lock(pendingMutex_);
            registrations = std::move(pendingQueue_);
            unregistrations = std::move(unregisterQueue_);
        }

        // 遅延登録を処理
        while (!registrations.empty()){
            Collider* c = registrations.front();
            registrations.pop();

            std::unique_lock lock(mutex_);
            colliders_[c->GetUniqueId()] = c;
        }

        // 遅延解除を処理
        while (!unregistrations.empty()){
            const Collider* c = unregistrations.front();
            unregistrations.pop();

            std::unique_lock lock(mutex_);
            colliders_.erase(c->GetUniqueId());

            auto it = std::ranges::remove_if(detectedPair_,
                                             [&c](const Pair& pair){
                return pair.first == c->GetUniqueId() || pair.second == c->GetUniqueId();
            });
        }
    }

    void Manager::Detect() {
        prePair_.clear();
        prePair_ = std::move(detectedPair_);
        detectedPair_.clear();

        // 処理前に遅延登録を適用
        ProcessPendingRegistrations();

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
        if (count == 0) return;

        std::vector<std::vector<Pair>> threadResults(maxThreadCount_);
        std::atomic<uint32_t> tasksCompleted = 0;
        uint32_t totalTasks = std::min(maxThreadCount_, static_cast<uint32_t>(count));
        const size_t chunkSize = std::max(1ULL, count / maxThreadCount_);

        EventTimer::GetInstance()->BeginEvent("Thread");
        // 各スレッドにタスクを割り当て
        for (uint32_t t = 0; t < totalTasks; ++t){
            const size_t start = t * chunkSize;
            const size_t end = std::min(start + chunkSize, count);
            const uint32_t threadIndex = t;

            AddTask([this, &array, &threadResults, start, end, threadIndex, &tasksCompleted](){
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

                threadResults[threadIndex] = std::move(localResults);
                ++tasksCompleted;
            });
        }

        // すべてのタスクが完了するのを待つ
        while (tasksCompleted < totalTasks){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        EventTimer::GetInstance()->EndEvent("Thread");
        // 結果をマージ
        {
            std::unique_lock lock(mutex_);
            for (const auto& results : threadResults){
                for (const auto& pair : results){
                    detectedPair_.push_back(pair);
                }
            }
        }
    }

    /**
     * メインスレッドで実行されることを前提としたProcessEventメソッド
     */
    void Manager::ProcessEvent() {
        // 処理中フラグを立てる
        isProcessingCollisions_ = true;

        std::unique_lock lock(mutex_);

        // 新規衝突の検出と継続衝突の処理
        for (const auto& pair : detectedPair_){
            auto itr = colliders_.find(pair.first);
            auto otr = colliders_.find(pair.second);

            if (itr == colliders_.end() || otr == colliders_.end()) continue;

            const auto& c1 = itr->second;
            const auto& c2 = otr->second;
            if (c1 == c2) continue;

            // 前回のペアから探す
            bool isNewCollision = true;
            for (const auto& pre : prePair_){
                if ((pre.first == pair.first && pre.second == pair.second) ||
                    (pre.first == pair.second && pre.second == pair.first)){
                    isNewCollision = false;
                    break;
                }
            }

            // メインスレッドでコールバック実行
            // ロックを一時的に解放
            lock.unlock();

            if (isNewCollision){
                // 新規衝突
                c1->OnCollision({EventType::Trigger, c2});
                c2->OnCollision({EventType::Trigger, c1});
            } else{
                // 継続衝突
                c1->OnCollision({EventType::Stay, c2});
                c2->OnCollision({EventType::Stay, c1});
            }

            // ロックを再取得
            lock.lock();
        }

        // 終了した衝突の処理
        for (const auto& pre : prePair_){
            auto itr = colliders_.find(pre.first);
            auto otr = colliders_.find(pre.second);

            if (itr == colliders_.end() || otr == colliders_.end()) continue;

            const auto& c1 = itr->second;
            const auto& c2 = otr->second;

            // 現在の衝突ペアから探す
            bool stillColliding = false;
            for (const auto& curr : detectedPair_){
                if ((curr.first == pre.first && curr.second == pre.second) ||
                    (curr.first == pre.second && curr.second == pre.first)){
                    stillColliding = true;
                    break;
                }
            }

            // 衝突が終了した場合
            if (!stillColliding){
                // ロックを一時的に解放してコールバック実行
                lock.unlock();

                c1->OnCollision({EventType::Exit, c2});
                c2->OnCollision({EventType::Exit, c1});

                // ロックを再取得
                lock.lock();
            }
        }

        // ロックを解放
        lock.unlock();

        // 遅延登録を処理
        ProcessPendingRegistrations();

        // 処理終了フラグを下げる
        isProcessingCollisions_ = false;
    }

    Manager::RayHitData Manager::RayCast(const Ray* _ray) {
        if (!_ray) return {};
        std::shared_lock lock(mutex_);

        RayHitData closestData {};
        float closestDistance = std::numeric_limits<float>::max();
        hitRaysOrderedByDistance_.clear();
        hitRays_.clear();

        for (const auto& value : colliders_ | std::views::values){
            if (!value->IsEnabled())continue;
            if (!Filter(_ray->GetData(), value->GetData()))continue;

            Detect(_ray, value);
        }

        if (hitRays_.empty())return {.uuid= "", .hitPoint= _ray->GetOrigin() + _ray->GetDirection() * _ray->GetLength()};

    	for (auto& data : hitRays_){
            float distance = (_ray->GetOrigin() - data.hitPoint).Length();
            data.distance = distance;

            if (distance < closestDistance){
                closestDistance = distance;
                closestData = data;
            }

            hitRaysOrderedByDistance_[distance] = data;
    	}
        return closestData;
    }

    Manager::RayHitData Manager::GetNextClosestHitData(float _distance)
    {
        for (auto& data : hitRaysOrderedByDistance_)
        {
            if (data.first > _distance) return data.second;
        }
        return RayHitData();
    }

    Collider* Manager::Get(const std::string& uuid) {
        if (!colliders_.contains(uuid))return nullptr;

        return colliders_[uuid];
    }

    bool Manager::Filter(const Pair& pair) const {
        const auto itr = colliders_.find(pair.first);
        const auto otr = colliders_.find(pair.second);

        if (itr == colliders_.end() || otr == colliders_.end()) return false;

        const Collider* c1 = itr->second;
        const Collider* c2 = otr->second;
        if (c1 == c2) return false;
        if (!c1->IsEnabled() || !c2->IsEnabled()) return false;
        if (c1->GetType() == Type::None || c2->GetType() == Type::None) return false;
    	if (c1->GetAttribute() & c2->GetIgnore() || c1->GetIgnore() & c2->GetAttribute()) return false;
        return true;
    }

    bool Manager::Filter(const Data& data, const Data& other) {
        if (data.uuid == other.uuid)return false;
        if (data.type == Type::None || other.type == Type::None)return false;
        if (data.attribute & other.ignore || data.ignore & other.attribute) return false;
        return true;
    }


    bool Manager::Detect(const Collider* c1, const Collider* c2) {
        float distance = (c1->GetTranslate() - c2->GetTranslate()).Length();
        if (100.f < distance)return false;

    	bool sp1 = std::holds_alternative<float>(c1->GetSize());
        bool sp2 = std::holds_alternative<float>(c2->GetSize());
        if (sp1 && sp2){
            // Sphere vs Sphere
            return distance <= (std::get<float>(c1->GetSize()) + std::get<float>(c2->GetSize()));
        } 
        if (!sp1 && !sp2){
            // AABB vs AABB
            const auto& min1 = c1->GetTranslate() - std::get<Vec3>(c1->GetSize()) * 0.5f;
            const auto& max1 = c1->GetTranslate() + std::get<Vec3>(c1->GetSize()) * 0.5f;
            const auto& min2 = c2->GetTranslate() - std::get<Vec3>(c2->GetSize()) * 0.5f;
            const auto& max2 = c2->GetTranslate() + std::get<Vec3>(c2->GetSize()) * 0.5f;

            return (min1.x <= max2.x && max1.x >= min2.x) &&
                (min1.y <= max2.y && max1.y >= min2.y) &&
                (min1.z <= max2.z && max1.z >= min2.z);
        }
        // AABB vs Sphere
        const auto& aabb = sp1 ? c2 : c1;
        const auto& sphere = sp1 ? c1 : c2;
        const auto& aabbSize = std::get<Vec3>(static_cast<const Collider*>(aabb)->GetSize());
        const auto& aabbTranslate = aabb->GetTranslate();
        const auto& aabbMin = aabbTranslate - (aabbSize/2.f);
        const auto& aabbMax = aabbTranslate + (aabbSize/2.f);
        const auto& sphereSize = std::get<float>(static_cast<const Collider*>(sphere)->GetSize());
        const auto& sphereTranslate = sphere->GetTranslate();

        return (sphereTranslate.x >= aabbMin.x - sphereSize && sphereTranslate.x <= aabbMax.x + sphereSize) &&
            (sphereTranslate.y >= aabbMin.y - sphereSize && sphereTranslate.y <= aabbMax.y + sphereSize) &&
            (sphereTranslate.z >= aabbMin.z - sphereSize && sphereTranslate.z <= aabbMax.z + sphereSize);
    }

    void Manager::Detect(const Ray* ray, const Collider* collider) {
        
    	if (collider->GetType() == Type::AABB){
            RayAABB(ray, collider);
            return;
        }
        
        RaySphere(ray, collider);
    }

    void Manager::RayAABB(const Ray* ray, const Collider* collider) {
        const Vec3& dir = ray->GetDirection();
        const Vec3& origin = ray->GetOrigin();
        const Vec3& center = collider->GetTranslate();
        const Vec3& halfSize = std::get<Vec3>(collider->GetSize()) * 0.5f;

        Vec3 t1 = (center - halfSize - origin) / dir;
        Vec3 t2 = (center + halfSize - origin) / dir;

        Vec3 tminVec = {
			std::min(t1.x, t2.x),
			std::min(t1.y, t2.y),
        	std::min(t1.z, t2.z),
		};

        Vec3 tmaxVec = {
            std::max(t1.x, t2.x),
            std::max(t1.y, t2.y),
            std::max(t1.z, t2.z),
        };

        float tmin = std::max({tminVec.x, tminVec.y, tminVec.z});
        float tmax = std::min({tmaxVec.x, tmaxVec.y, tmaxVec.z});

        if (tmin > tmax || tmax < 0.0f) return;

        float t = (tmin >= 0.0f) ? tmin : tmax;
        if (0.0f <= t && t <= ray->GetLength()) {
            RayHitData hitData {
                .uuid = collider->GetUniqueId(),
                .hitPoint = ray->GetPoint(t)
            };
            hitRays_.push_back(hitData);
        }
    }

    void Manager::RaySphere(const Ray* ray, const Collider* collider) {
        // レイの原点からコライダーの中心へのベクトル
        float dx = collider->GetTranslate().x - ray->GetOrigin().x;
        float dy = collider->GetTranslate().y - ray->GetOrigin().y;
        float dz = collider->GetTranslate().z - ray->GetOrigin().z;

        // レイの方向ベクトル上でのコライダー中心への射影
        float projection_length = dx * ray->GetDirection().x + dy * ray->GetDirection().y + dz * ray->GetDirection().z;

        // レイの後ろにコライダーがある場合は衝突なし
        if (projection_length < 0){
            return;
        }

        // レイの最大長より遠い場合も衝突なし
        if (projection_length > ray->GetLength()){
            return;
        }

        // 射影点からコライダー中心までの距離の2乗
        float d2 = dx * dx + dy * dy + dz * dz - projection_length * projection_length;

        // コライダーの半径の2乗
        float r2;
        if (std::holds_alternative<float>(collider->GetSize())){
            r2 = std::get<float>(collider->GetSize()) * std::get<float>(collider->GetSize());
        } else{
            r2 = std::get<Vec3>(collider->GetSize()).x;
            r2 *= r2;
        }

        // 距離が半径より大きければ衝突なし
        if (d2 > r2){
            return;
        }

        // ここまで来れば衝突している
        // 衝突点までの距離を計算
        float t = projection_length - std::sqrt(r2 - d2);

        // レイの範囲内で最も近い衝突点を計算
        if (t < 0){
            t = projection_length - sqrtf(r2 - d2);

            if (t < 0){
                t = 0.f;
            }
        }

        t = std::min(t, ray->GetLength());

        // 衝突点の座標を計算
        Vec3 hit_point = ray->GetPoint(t);

        // 衝突データを作成
        RayHitData hitData {.uuid = collider->GetUniqueId(), .hitPoint = hit_point};
        hitRays_.push_back(hitData);
    }
}
