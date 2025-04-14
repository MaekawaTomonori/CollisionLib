#pragma once
#include <shared_mutex>
#include <thread>
#include <atomic>
#include <vector>

#include "Collider.h"

namespace Collision{
    class Manager{
        using Pair = std::pair<std::string, std::string>;
        // 登録済みコライダー情報
        std::unordered_map<std::string, Collider*> colliders_;
        // 衝突確認済みペア
        std::vector<Pair> detectedPair_;
        std::vector<Pair> prePair_;

        std::shared_mutex mutex_;
        uint32_t maxThreadCount_ {std::thread::hardware_concurrency()};

        // スレッドプール関連
        std::vector<std::thread> threadPool_;
        std::atomic<bool> running_ {true};

        // 直接登録するためのフラグ
        std::atomic<bool> isProcessingCollisions_ {false};

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
         */
        void Detect();

        /**
         * 衝突イベントを処理します。
         */
        void ProcessEvent();

        /**
         * 衝突処理中かどうかを返します。
         * @return 衝突処理中の場合はtrue
         */
        bool IsProcessingCollisions() const {
            return isProcessingCollisions_;
        }

        private:
        /**
         * スレッドプールを初期化します。
         */
        void InitThreadPool();

        /**
         * ワーカースレッドの処理を行います。
         * @param threadId スレッドID
         */
        void WorkerThread(int threadId) const;

        /**
         * ペアがフィルター条件に一致するか確認します。
         * @param pair 確認するペア
         * @return フィルター条件に一致する場合はtrue
         */
        bool Filter(const Pair& pair) const;

        /**
         * 2つのコライダー間の衝突を検出します。
         * @param c1 1つ目のコライダー
         * @param c2 2つ目のコライダー
         * @return 衝突している場合はtrue
         */
        static bool Detect(const Collider* c1, const Collider* c2);
    };
}

