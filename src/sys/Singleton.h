#pragma once
#include <cassert>
#include <mutex>

/**
 * Res:mozc(https://github.com/google/mozc/blob/afb03ddfe72dde4cf2409863a3bfea160f7a66d8/src/base/singleton.h)
 *  Qiita(https://qiita.com/kikuuuty/items/fcf5f7df2f0493c437dc)
 */
class SingletonFinalizer{
	public:
	using Finalizer = void(*)();
	static void AddFinalizer(Finalizer finalizer);
	static void Finalize();
};

template <typename T>
class Singleton final{
	static T* instance_;
	static std::once_flag flag_;

	public:
	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;

	static T* Get();

	private:
	Singleton() = default;
	~Singleton() = default;

	static void Create();
	static void Destroy();
};

template <typename T> T* Singleton<T>::instance_ = nullptr;
template <typename T> std::once_flag Singleton<T>::flag_;

template <typename T>
T* Singleton<T>::Get() {
	std::call_once(flag_, Create);
	assert(instance_);
	return instance_;
}

template <typename T>
void Singleton<T>::Create() {
	instance_ = new T;
	SingletonFinalizer::AddFinalizer(&Destroy);
}

template <typename T>
void Singleton<T>::Destroy() {
	delete instance_;
	instance_ = nullptr;
}

