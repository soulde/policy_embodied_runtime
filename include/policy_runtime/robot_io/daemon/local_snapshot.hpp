#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <type_traits>

namespace policy_runtime {

// Fixed-capacity, single-writer publication for in-process RT handoff. Payload
// words are atomic so a reader racing two publications never accesses ordinary
// storage concurrently. The bounded reader either returns one complete version
// or no value; it never locks, waits, or allocates.
template <class T>
class LocalSnapshot {
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(sizeof(T) % sizeof(std::uint64_t) == 0U);

 public:
  void publish(const T& value) noexcept {
    const auto active = active_index_.load(std::memory_order_seq_cst);
    const auto target = active < slots_.size() ? 1U - active : 0U;
    auto& slot = slots_[target];
    const auto epoch = epoch_.fetch_add(2U, std::memory_order_seq_cst) + 2U;
    slot.guard.store(epoch | 1U, std::memory_order_seq_cst);

    std::array<std::byte, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
    for (std::size_t word_index = 0; word_index < kWordCount; ++word_index) {
      std::uint64_t word{};
      std::memcpy(&word, bytes.data() + word_index * sizeof(word), sizeof(word));
      slot.words[word_index].store(word, std::memory_order_seq_cst);
    }
    slot.guard.store(epoch, std::memory_order_seq_cst);
    active_index_.store(target, std::memory_order_seq_cst);
  }

  std::optional<T> read() const noexcept {
    for (unsigned attempt = 0; attempt < 16U; ++attempt) {
      const auto active = active_index_.load(std::memory_order_seq_cst);
      if (active >= slots_.size()) {
        return std::nullopt;
      }
      const auto& slot = slots_[active];
      const auto before = slot.guard.load(std::memory_order_seq_cst);
      if ((before & 1U) != 0U) {
        continue;
      }
      std::array<std::byte, sizeof(T)> bytes{};
      for (std::size_t word_index = 0; word_index < kWordCount; ++word_index) {
        const auto word = slot.words[word_index].load(std::memory_order_seq_cst);
        std::memcpy(bytes.data() + word_index * sizeof(word), &word, sizeof(word));
      }
      const auto after = slot.guard.load(std::memory_order_seq_cst);
      const auto confirmed_active =
          active_index_.load(std::memory_order_seq_cst);
      if (before != after || (after & 1U) != 0U || confirmed_active != active) {
        continue;
      }
      T value{};
      std::memcpy(&value, bytes.data(), sizeof(T));
      return value;
    }
    return std::nullopt;
  }

  bool atomics_are_lock_free() const noexcept {
    if (!active_index_.is_lock_free() || !epoch_.is_lock_free()) {
      return false;
    }
    for (const auto& slot : slots_) {
      if (!slot.guard.is_lock_free()) {
        return false;
      }
      for (const auto& word : slot.words) {
        if (!word.is_lock_free()) {
          return false;
        }
      }
    }
    return true;
  }

 private:
  static constexpr std::size_t kWordCount = sizeof(T) / sizeof(std::uint64_t);
  static constexpr std::uint32_t kNoActiveSlot =
      std::numeric_limits<std::uint32_t>::max();

  struct alignas(64) Slot {
    std::atomic<std::uint64_t> guard{};
    std::array<std::atomic<std::uint64_t>, kWordCount> words{};
  };

  std::array<Slot, 2> slots_{};
  alignas(64) std::atomic<std::uint32_t> active_index_{kNoActiveSlot};
  std::atomic<std::uint64_t> epoch_{};
};

}  // namespace policy_runtime
