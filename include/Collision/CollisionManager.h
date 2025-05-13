#pragma once
#include <shared_mutex>
#include <thread>
#include <atomic>
#include <queue>
#include <vector>

#include "Collider.h"

namespace Collision{
    class Manager{
    public:
    	using Pair = std::pair<std::string, std::string>;

        struct RayHitData{
            Pair pair;
            Vec3 hitPoint;
        };

    private:
        // 登録済みコライダー情報
        std::unordered_map<std::string, Collider*> colliders_;
        // 衝突確認済みペア
        std::vector<Pair> detectedPair_;
        std::vector<Pair> prePair_;

        std::shared_mutex mutex_;
        uint32_t maxThreadCount_ {std::thread::hardware_concurrency()};

        // スレッドプール関連
        std::vector<std::thread> threadPool_;
        std::queue<std::function<void()>>tasks_;
        std::mutex taskMutex_;
        std::condition_variable taskCondition_;
        std::atomic<bool> running_ {true};

        // 直接登録するためのフラグ
        std::atomic<bool> isProcessingCollisions_ {false};

        std::queue<Collider*> pendingQueue_;
        std::queue<const Collider*> unregisterQueue_;
        std::mutex pendingMutex_;

        std::vector<RayHitData> hitRays_;
    public:
        Manager();
        ~Manager();

        /**
         * コライダーを登録します。
         * @param collider 登録するコライダー
         * @return 登録成功時はtrue
         */
        bool Register(Collider* collider);

        /**
         * コライダーの登録を解除します。
         * @param collider 登録解除するコライダー
         * @return 解除成功時はtrue
         */
        bool Unregister(const Collider* collider);

        /**
         * 衝突検出を実行します。
         * この処理はスレッドプールを使用して並列に実行されます。
         */
        void Detect();

        /**
         * 衝突イベントを処理します。
         * この処理はメインスレッドで実行されます。
         */
        void ProcessEvent();

        RayHitData RayCast(const Ray* _ray);

        Collider* Get(const std::string& uuid);
    private:

        void  ProcessPendingRegistrations();
        void AddTask(std::function<void()> task);
        void WaitForTasks();

    	/**
         * スレッドプールを初期化します。
         */
        void InitThreadPool();
        void WorkerThread();

        /**
         * ペアがフィルター条件に一致するか確認します。
         * @param pair 確認するペア
         * @return フィルター条件に一致する場合はtrue
         */
        bool Filter(const Pair& pair) const;

	    static bool Filter(const Data& data, const Data& other);

        /**
         * 2つのコライダー間の衝突を検出します。
         * @param c1 1つ目のコライダー
         * @param c2 2つ目のコライダー
         * @return 衝突している場合はtrue
         */
        static bool Detect(const Collider* c1, const Collider* c2);
	    void Detect(const Ray* ray, const Collider* collider);
    };
}