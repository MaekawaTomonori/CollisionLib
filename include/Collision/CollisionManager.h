#pragma once
#include <shared_mutex>
#include <thread>
#include <queue>
#include <functional>
#include <condition_variable>
#include <atomic>

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
		std::queue<std::function<void()>> taskQueue_;
		std::mutex queueMutex_;
		std::condition_variable condition_;
		std::atomic<bool> running_ {true};

	public:
		Manager();
		~Manager();

		bool Register(Collider* c);
		bool Unregister(const Collider* c);

		void Detect();
		void ProcessEvent();

	private:
		void InitThreadPool();
		void WorkerThread();
		void AddTask(std::function<void()> task);
		void WaitForTasks();
		
		bool Filter(const Pair& _pair) const;
		static bool Detect(const Collider* _c1, const Collider* _c2);
	};
}

