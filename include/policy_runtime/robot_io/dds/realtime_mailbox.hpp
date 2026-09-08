#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <type_traits>

namespace policy_runtime::robot_io::dds {
namespace detail {

template <class T>
class AtomicValue {
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::atomic<std::uint8_t>::is_always_lock_free);

 public:
  void store(const T& value) noexcept {
    const auto next = version_.load(std::memory_order_relaxed) + 1U;
    version_.store(next, std::memory_order_release);
    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
    for (std::size_t index = 0; index < bytes.size(); ++index) {
      bytes_[index].store(bytes[index], std::memory_order_relaxed);
    }
    version_.store(next + 1U, std::memory_order_release);
  }

  std::optional<T> load() const noexcept {
    for (unsigned attempt = 0; attempt < 16U; ++attempt) {
      const auto before = version_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U || before == 0U) {
        continue;
      }
      std::array<std::uint8_t, sizeof(T)> bytes{};
      for (std::size_t index = 0; index < bytes.size(); ++index) {
        bytes[index] = bytes_[index].load(std::memory_order_relaxed);
      }
      const auto after = version_.load(std::memory_order_acquire);
      if (before != after || (after & 1U) != 0U) {
        continue;
      }
      T value{};
      std::memcpy(&value, bytes.data(), sizeof(T));
      return value;
    }
    return std::nullopt;
  }

 private:
  std::atomic<std::uint64_t> version_{};
  std::array<std::atomic<std::uint8_t>, sizeof(T)> bytes_{};
};

}  // namespace detail

template <class T, std::size_t Capacity>
class SensorEventQueue {
  static_assert(Capacity > 0U);
  static_assert(std::is_trivially_copyable_v<T>);

  struct StoredEvent {
    std::uint64_t ticket{};
    T value{};
  };

 public:
  bool push_realtime(const T& value) noexcept {
    const auto published = published_.load(std::memory_order_relaxed);
    const auto consumed = consumed_.load(std::memory_order_acquire);
    if (published - consumed >= Capacity) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
    }
    slots_[published % Capacity].store(StoredEvent{published + 1U, value});
    published_.store(published + 1U, std::memory_order_release);
    return true;
  }

  std::optional<T> pop() noexcept {
    const auto published = published_.load(std::memory_order_acquire);
    auto consumed = consumed_.load(std::memory_order_relaxed);
    if (consumed >= published) {
      return std::nullopt;
    }
    const auto oldest = published > Capacity ? published - Capacity : 0U;
    if (consumed < oldest) {
      consumed = oldest;
    }
    const auto event = slots_[consumed % Capacity].load();
    if (!event.has_value() || event->ticket != consumed + 1U) {
      return std::nullopt;
    }
    consumed_.store(consumed + 1U, std::memory_order_release);
    return event->value;
  }

  std::uint64_t dropped() const noexcept {
    return dropped_.load(std::memory_order_relaxed);
  }

 private:
  std::array<detail::AtomicValue<StoredEvent>, Capacity> slots_{};
  alignas(64) std::atomic<std::uint64_t> published_{};
  alignas(64) std::atomic<std::uint64_t> consumed_{};
  std::atomic<std::uint64_t> dropped_{};
};

template <class T>
class CommandMailbox {
  static_assert(std::is_trivially_copyable_v<T>);

 public:
  void publish(const T& value) noexcept { value_.store(value); }

  std::optional<T> read_latest(std::uint64_t after_sequence) const noexcept {
    auto value = value_.load();
    if (!value.has_value() || value->sequence <= after_sequence) {
      return std::nullopt;
    }
    return value;
  }

 private:
  detail::AtomicValue<T> value_;
};

template <class T>
struct EpochCommand {
  std::uint64_t epoch{};
  T command{};
};

template <class T, std::size_t MaximumMembers>
struct EthercatEpochBatch {
  std::uint64_t epoch{};
  std::size_t count{};
  std::array<T, MaximumMembers> commands{};
};

template <class T, std::size_t MaximumMembers>
class EthercatEpochMailbox {
  static_assert(MaximumMembers > 0U);
  static_assert(std::is_trivially_copyable_v<T>);

  using Batch = EthercatEpochBatch<T, MaximumMembers>;

 public:
  explicit EthercatEpochMailbox(std::size_t member_count) noexcept
      : size_(member_count <= MaximumMembers ? member_count : 0U) {}

  bool stage(std::size_t index, const EpochCommand<T>& value) noexcept {
    if (size_ == 0U || index >= size_ || value.epoch == 0U ||
        value.epoch < staging_epoch_) {
      return false;
    }
    if (value.epoch != staging_epoch_) {
      staging_epoch_ = value.epoch;
      staged_.fill(false);
    }
    commands_[index] = value.command;
    staged_[index] = true;
    return true;
  }

  bool commit(std::uint64_t epoch) noexcept {
    if (size_ == 0U || epoch == 0U || epoch != staging_epoch_) {
      return false;
    }
    for (std::size_t index = 0; index < size_; ++index) {
      if (!staged_[index]) {
        return false;
      }
    }
    Batch batch{};
    batch.epoch = epoch;
    batch.count = size_;
    batch.commands = commands_;
    committed_.store(batch);
    staged_.fill(false);
    return true;
  }

  std::optional<Batch> consume_complete(
      std::uint64_t after_epoch = 0U) const noexcept {
    auto batch = committed_.load();
    if (!batch.has_value() || batch->epoch <= after_epoch) {
      return std::nullopt;
    }
    return batch;
  }

 private:
  std::size_t size_{};
  std::uint64_t staging_epoch_{};
  std::array<T, MaximumMembers> commands_{};
  std::array<bool, MaximumMembers> staged_{};
  detail::AtomicValue<Batch> committed_;
};

}  // namespace policy_runtime::robot_io::dds
