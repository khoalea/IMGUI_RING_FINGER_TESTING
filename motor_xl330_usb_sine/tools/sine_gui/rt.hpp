// rt.hpp — lock-free primitives + thread placement. No ImGui, no owon_spe: testable alone.
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <type_traits>
#include <pthread.h>
#include <sched.h>

namespace rt {

inline constexpr std::size_t kCacheLine = 64;

// Single-producer / single-consumer ring. Wait-free on both sides, no allocation after ctor.
// One queue per (producer thread, consumer thread) pair — never share an end between threads.
template <class T>
class SpscQueue {
  static_assert(std::is_trivially_copyable_v<T>, "POD only: no std::string/vector across threads");
public:
  explicit SpscQueue(std::size_t capacity_pow2)
      : mask_(capacity_pow2 - 1), buf_(std::make_unique<T[]>(capacity_pow2)) {
    if (capacity_pow2 < 2 || (capacity_pow2 & mask_) != 0) std::abort();
  }
  bool try_push(const T& v) noexcept {                    // producer only
    const auto h = head_.load(std::memory_order_relaxed);
    if (h - tail_.load(std::memory_order_acquire) > mask_) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    buf_[h & mask_] = v;
    head_.store(h + 1, std::memory_order_release);
    return true;
  }
  bool try_pop(T& out) noexcept {                         // consumer only
    const auto t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire)) return false;
    out = buf_[t & mask_];
    tail_.store(t + 1, std::memory_order_release);
    return true;
  }
  std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
private:
  const std::size_t mask_;
  std::unique_ptr<T[]> buf_;
  alignas(kCacheLine) std::atomic<std::uint64_t> head_{0};
  alignas(kCacheLine) std::atomic<std::uint64_t> tail_{0};
  alignas(kCacheLine) std::atomic<std::uint64_t> dropped_{0};
};

// Latest-value mailbox (triple buffer). Writer never blocks, reader always gets a whole,
// most-recent T. Use for state where only "now" matters, not history.
template <class T>
class Latest {
  static_assert(std::is_trivially_copyable_v<T>);
public:
  void write(const T& v) noexcept {                       // single writer
    buf_[w_] = v;
    w_ = static_cast<std::uint8_t>(
        shared_.exchange(static_cast<std::uint8_t>(w_ | kDirty), std::memory_order_acq_rel) & kIdx);
  }
  bool read(T& out) noexcept {                            // single reader; false = nothing new
    if (!(shared_.load(std::memory_order_acquire) & kDirty)) return false;
    r_ = static_cast<std::uint8_t>(shared_.exchange(r_, std::memory_order_acq_rel) & kIdx);
    out = buf_[r_];
    return true;
  }
private:
  static constexpr std::uint8_t kDirty = 4, kIdx = 3;
  T buf_[3]{};
  alignas(kCacheLine) std::atomic<std::uint8_t> shared_{1};
  alignas(kCacheLine) std::uint8_t w_ = 0;
  alignas(kCacheLine) std::uint8_t r_ = 2;
};

// ---- thread placement (Linux) ---------------------------------------------------------------
inline bool pin_this_thread(int core) noexcept {
  cpu_set_t set; CPU_ZERO(&set); CPU_SET(core, &set);
  return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
}
inline void name_this_thread(const char* name) noexcept { pthread_setname_np(pthread_self(), name); }
inline int current_core() noexcept { return sched_getcpu(); }

} // namespace rt
