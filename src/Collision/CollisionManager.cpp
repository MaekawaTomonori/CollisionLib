#include "Collision/CollisionManager.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <queue>
#include <functional>
#include <ranges>
#include <type_traits>
#include <unordered_map>

#include "Math/MathUtils.hpp"

namespace Collision{
    namespace{
        /**
         * コライダーのtranslate(Capsuleなら始点)から その形状が到達しうる最大距離を返す
         * 広域カリングの閾値を形状ごとに正しく求めるために使う
         */
        float BoundingRadius(const Collider* _collider) {
            return std::visit([](const auto& _shape) -> float {
                using Shape = std::decay_t<decltype(_shape)>;
                if constexpr (std::is_same_v<Shape, SphereShape>){
                    return _shape.radius;
                } else if constexpr (std::is_same_v<Shape, AabbShape>){
                    // 中心から最も遠い頂点までの距離(対角線の半分)
                    return _shape.size.Length() * 0.5f;
                } else if constexpr (std::is_same_v<Shape, CapsuleShape>){
                    // 始点から最も遠い 終点側キャップの表面までの距離
                    return _shape.offset.Length() + _shape.radius;
                }
            }, _collider->GetSize());
        }

        /**
         * ブロードフェーズ用の均一グリッドのセル座標
         */
        struct CellCoord{
            int32_t x, y, z;
            bool operator==(const CellCoord&) const = default;
        };

        struct CellCoordHash{
            size_t operator()(const CellCoord& _coord) const noexcept {
                size_t h = std::hash<int32_t>{}(_coord.x);
                h ^= std::hash<int32_t>{}(_coord.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
                h ^= std::hash<int32_t>{}(_coord.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
                return h;
            }
        };

        CellCoord ToCellCoord(const Vector3& _pos, float _cellSize) {
            return CellCoord{
                static_cast<int32_t>(std::floor(_pos.x / _cellSize)),
                static_cast<int32_t>(std::floor(_pos.y / _cellSize)),
                static_cast<int32_t>(std::floor(_pos.z / _cellSize))
            };
        }

        /**
         * 隣接26セルのうち前方13方向のみを辿る
         * 各セルペアを1度だけ処理するための一方向オフセット
         */
        constexpr std::array<CellCoord, 13> kForwardCellOffsets = {{
            {1,-1,-1},{1,-1,0},{1,-1,1},
            {1, 0,-1},{1, 0,0},{1, 0,1},
            {1, 1,-1},{1, 1,0},{1, 1,1},
            {0, 1,-1},{0, 1,0},{0, 1,1},
            {0, 0, 1}
        }};

        bool DetectCapsuleSphere(const Collider* _capsule, const Collider* _sphere) {
            const auto capsuleShape = std::get<CapsuleShape>(_capsule->GetSize());
            const Vector3 start = _capsule->GetTranslate();
            const Vector3 end = start + capsuleShape.offset;

            const Vector3 closest = MathUtils::ClosestPointOnSegment(_sphere->GetTranslate(), start, end);
            const float radiusSum = capsuleShape.radius + std::get<SphereShape>(_sphere->GetSize()).radius;

            return MathUtils::SquaredDistance(closest, _sphere->GetTranslate()) <= radiusSum * radiusSum;
        }

        /**
         * カプセルとAABBの最近傍点を反復法(alternating projection)で近似的に求める。
         * 線分・AABBはどちらも凸形状なので、数回の反復で実用上十分収束する。
         */
        bool DetectCapsuleAabb(const Collider* _capsule, const Collider* _aabb) {
            const auto capsuleShape = std::get<CapsuleShape>(_capsule->GetSize());
            const Vector3 start = _capsule->GetTranslate();
            const Vector3 end = start + capsuleShape.offset;

            const Vector3 aabbSize = std::get<AabbShape>(_aabb->GetSize()).size;
            const Vector3 aabbCenter = _aabb->GetTranslate();
            const Vector3 aabbMin = aabbCenter - aabbSize * 0.5f;
            const Vector3 aabbMax = aabbCenter + aabbSize * 0.5f;

            Vector3 pointOnSegment = start;
            Vector3 pointOnBox = MathUtils::Clamp(pointOnSegment, aabbMin, aabbMax);
            for (int i = 0; i < 4; ++i){
                pointOnSegment = MathUtils::ClosestPointOnSegment(pointOnBox, start, end);
                pointOnBox = MathUtils::Clamp(pointOnSegment, aabbMin, aabbMax);
            }

            return MathUtils::SquaredDistance(pointOnSegment, pointOnBox) <= capsuleShape.radius * capsuleShape.radius;
        }

        bool DetectCapsuleCapsule(const Collider* _c1, const Collider* _c2) {
            const auto shape1 = std::get<CapsuleShape>(_c1->GetSize());
            const auto shape2 = std::get<CapsuleShape>(_c2->GetSize());

            const Vector3 start1 = _c1->GetTranslate();
            const Vector3 end1 = start1 + shape1.offset;
            const Vector3 start2 = _c2->GetTranslate();
            const Vector3 end2 = start2 + shape2.offset;

            const float radiusSum = shape1.radius + shape2.radius;
            return MathUtils::SquaredDistanceBetweenSegments(start1, end1, start2, end2) <= radiusSum * radiusSum;
        }
    }

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

        // BoundingRadiusはコライダーのサイズのみに依存するため ペアごとではなく
        // コライダーごとに1回だけ計算してキャッシュする(O(n^2)ではなくO(n)にするため)
        std::vector<std::tuple<std::string, Collider*, float>> array;
        float maxRadius = 0.f;
        {
            std::shared_lock lock(mutex_);
            for (const auto& [key, value] : colliders_){
                if (value->IsEnabled()){
                    const float radius = BoundingRadius(value);
                    maxRadius = std::max(maxRadius, radius);
                    array.emplace_back(key, value, radius);
                }
            }
        }

        const size_t count = array.size();
        if (count == 0) return;

        // 均一グリッドによるブロードフェーズ
        // セルサイズを最大BoundingRadiusの2倍にしておくと 衝突しうるペアは必ず
        // 同じセルか前方13方向の隣接セルのどちらかに収まる
        const float cellSize = std::max(maxRadius * 2.f, 0.01f);

        std::unordered_map<CellCoord, std::vector<size_t>, CellCoordHash> grid;
        grid.reserve(count);
        for (size_t i = 0; i < count; ++i){
            const CellCoord coord = ToCellCoord(std::get<1>(array[i])->GetTranslate(), cellSize);
            grid[coord].push_back(i);
        }

        std::vector<CellCoord> cells;
        cells.reserve(grid.size());
        for (const auto& coord : grid | std::views::keys) cells.push_back(coord);

        const size_t cellCount = cells.size();

        std::vector<std::vector<Pair>> threadResults(maxThreadCount_);
        std::atomic<uint32_t> tasksCompleted = 0;
        uint32_t totalTasks = std::min(maxThreadCount_, static_cast<uint32_t>(cellCount));
        const size_t chunkSize = std::max(1ULL, cellCount / maxThreadCount_);

        // 各スレッドにセル単位でタスクを割り当て
        for (uint32_t t = 0; t < totalTasks; ++t){
            const size_t start = t * chunkSize;
            const size_t end = std::min(start + chunkSize, cellCount);
            const uint32_t threadIndex = t;

            AddTask([this, &array, &grid, &cells, start, end, threadIndex, &threadResults, &tasksCompleted](){
                std::vector<Pair> localResults;

                auto tryPair = [&array, &localResults](size_t _ia, size_t _ib){
                    const auto& [id1, c1, radius1] = array[_ia];
                    const auto& [id2, c2, radius2] = array[_ib];

                    if (!Filter(c1, c2)) return;
                    if (Detect(c1, radius1, c2, radius2)){
                        localResults.emplace_back(id1, id2);
                    }
                };

                for (size_t ci = start; ci < end; ++ci){
                    const CellCoord& coord = cells[ci];
                    const auto selfIt = grid.find(coord);
                    if (selfIt == grid.end()) continue; // 想定外だが例外を避けて安全側に倒す
                    const auto& members = selfIt->second;

                    // 同一セル内のペア
                    for (size_t a = 0; a < members.size(); ++a){
                        for (size_t b = a + 1; b < members.size(); ++b){
                            tryPair(members[a], members[b]);
                        }
                    }

                    // 前方13方向の隣接セルとのペア(各セルペアを1度だけ処理する)
                    for (const auto& offset : kForwardCellOffsets){
                        const CellCoord neighbor{coord.x + offset.x, coord.y + offset.y, coord.z + offset.z};
                        const auto it = grid.find(neighbor);
                        if (it == grid.end()) continue;

                        for (const size_t a : members){
                            for (const size_t b : it->second){
                                tryPair(a, b);
                            }
                        }
                    }
                }

                threadResults[threadIndex] = std::move(localResults);
                ++tasksCompleted;
            });
        }

        // すべてのタスクが完了するのを待つ
        // 1タスクの処理時間はミリ秒未満のため sleep_for(1ms)の固定待ちだと
        // その粒度自体が待ち時間の支配要因になってしまう。yieldでスピンウェイトする
        while (tasksCompleted < totalTasks){
            std::this_thread::yield();
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
        const auto it = hitRaysOrderedByDistance_.upper_bound(_distance);
        if (it == hitRaysOrderedByDistance_.end()) return {};

        return it->second;
    }

    Collider* Manager::Get(const std::string& uuid) {
        if (!colliders_.contains(uuid))return nullptr;

        return colliders_[uuid];
    }

    std::vector<const Collider*> Manager::GetAll() const {
        std::shared_lock lock(mutex_);
        std::vector<const Collider*> result;
        result.reserve(colliders_.size());
        for (const auto& c : colliders_ | std::views::values) result.push_back(c);
        return result;
    }

    bool Manager::Filter(const Collider* c1, const Collider* c2) {
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


    bool Manager::Detect(const Collider* c1, float radius1, const Collider* c2, float radius2) {
        // sqrtを避けるため二乗距離で比較する
        const float radiusSum = radius1 + radius2;
        if (MathUtils::SquaredDistance(c1->GetTranslate(), c2->GetTranslate()) > radiusSum * radiusSum) return false;

        const Type type1 = c1->GetType();
        const Type type2 = c2->GetType();

        if (type1 == Type::Sphere && type2 == Type::Sphere){
            const float shapeRadiusSum = std::get<SphereShape>(c1->GetSize()).radius + std::get<SphereShape>(c2->GetSize()).radius;
            return MathUtils::SquaredDistance(c1->GetTranslate(), c2->GetTranslate()) <= shapeRadiusSum * shapeRadiusSum;
        }
        if (type1 == Type::AABB && type2 == Type::AABB){
            const auto& min1 = c1->GetTranslate() - std::get<AabbShape>(c1->GetSize()).size * 0.5f;
            const auto& max1 = c1->GetTranslate() + std::get<AabbShape>(c1->GetSize()).size * 0.5f;
            const auto& min2 = c2->GetTranslate() - std::get<AabbShape>(c2->GetSize()).size * 0.5f;
            const auto& max2 = c2->GetTranslate() + std::get<AabbShape>(c2->GetSize()).size * 0.5f;

            return (min1.x <= max2.x && max1.x >= min2.x) &&
                (min1.y <= max2.y && max1.y >= min2.y) &&
                (min1.z <= max2.z && max1.z >= min2.z);
        }
        if ((type1 == Type::Sphere && type2 == Type::AABB) || (type1 == Type::AABB && type2 == Type::Sphere)){
            const auto& aabb = (type1 == Type::AABB) ? c1 : c2;
            const auto& sphere = (type1 == Type::Sphere) ? c1 : c2;

            const auto aabbSize = std::get<AabbShape>(aabb->GetSize()).size;
            const auto aabbTranslate = aabb->GetTranslate();
            const auto aabbMin = aabbTranslate - (aabbSize/2.f);
            const auto aabbMax = aabbTranslate + (aabbSize/2.f);
            const auto sphereSize = std::get<SphereShape>(sphere->GetSize()).radius;
            const auto sphereTranslate = sphere->GetTranslate();

            return (sphereTranslate.x >= aabbMin.x - sphereSize && sphereTranslate.x <= aabbMax.x + sphereSize) &&
                (sphereTranslate.y >= aabbMin.y - sphereSize && sphereTranslate.y <= aabbMax.y + sphereSize) &&
                (sphereTranslate.z >= aabbMin.z - sphereSize && sphereTranslate.z <= aabbMax.z + sphereSize);
        }
        if (type1 == Type::Capsule && type2 == Type::Capsule){
            return DetectCapsuleCapsule(c1, c2);
        }
        if ((type1 == Type::Capsule && type2 == Type::Sphere) || (type1 == Type::Sphere && type2 == Type::Capsule)){
            const auto& capsule = (type1 == Type::Capsule) ? c1 : c2;
            const auto& sphere = (type1 == Type::Sphere) ? c1 : c2;
            return DetectCapsuleSphere(capsule, sphere);
        }
        if ((type1 == Type::Capsule && type2 == Type::AABB) || (type1 == Type::AABB && type2 == Type::Capsule)){
            const auto& capsule = (type1 == Type::Capsule) ? c1 : c2;
            const auto& aabb = (type1 == Type::AABB) ? c1 : c2;
            return DetectCapsuleAabb(capsule, aabb);
        }
        return false;
    }

    void Manager::Detect(const Ray* ray, const Collider* collider) {
        switch (collider->GetType()){
            case Type::AABB:
                RayAABB(ray, collider);
                return;
            case Type::Sphere:
                RaySphere(ray, collider);
                return;
            default:
                // Ray vs Capsule は未対応。SphereShape/AabbShape用のGetterを
                // 誤って呼びbad_variant_accessを起こさないよう、ここで安全に弾く。
                return;
        }
    }

    void Manager::RayAABB(const Ray* ray, const Collider* collider) {
        const Vector3& dir = ray->GetDirection();
        const Vector3& origin = ray->GetOrigin();
        const Vector3& center = collider->GetTranslate();
        const Vector3& halfSize = std::get<AabbShape>(collider->GetSize()).size * 0.5f;

        Vector3 t1 = (center - halfSize - origin) / dir;
        Vector3 t2 = (center + halfSize - origin) / dir;

        Vector3 tminVec = {
			std::min(t1.x, t2.x),
			std::min(t1.y, t2.y),
        	std::min(t1.z, t2.z),
		};

        Vector3 tmaxVec = {
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
        if (std::holds_alternative<SphereShape>(collider->GetSize())){
            const float radius = std::get<SphereShape>(collider->GetSize()).radius;
            r2 = radius * radius;
        } else{
            r2 = std::get<AabbShape>(collider->GetSize()).size.x;
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
        Vector3 hit_point = ray->GetPoint(t);

        // 衝突データを作成
        RayHitData hitData {.uuid = collider->GetUniqueId(), .hitPoint = hit_point};
        hitRays_.push_back(hitData);
    }
}
