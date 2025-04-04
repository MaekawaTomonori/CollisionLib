#pragma once
#include <shared_mutex>
#include <unordered_set>

#include "Collider.h"

namespace Collision{
	class Manager{
		using Pair = std::pair<std::string, std::string>;
		// �o�^�ς݃R���C�_�[���
		std::unordered_map<std::string, Collider*> colliders_;
		// �Փˊm�F�ς݃y�A
		std::vector<Pair> detectedPair_;
		std::vector<Pair> prePair_;

		std::shared_mutex mutex_;
		uint32_t maxThreadCount_ {std::thread::hardware_concurrency()};

	public:
		bool Register(Collider* c);
		bool Unregister(const Collider* c);

		void Detect();
		void ProcessEvent();

	private:
		bool Filter(const Pair& _pair) const;
		static bool Detect(const Collider* _c1, const Collider* _c2);
	};
}

