#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <type_traits>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/robot_io/ipc_protocol.hpp"

namespace policy_runtime {

namespace detail {

inline constexpr std::size_t kCacheLineSize = 64;
inline constexpr std::uint32_t kNoActiveSlot = std::numeric_limits<std::uint32_t>::max();
inline constexpr unsigned int kSnapshotReadAttempts = 8;

struct alignas(kCacheLineSize) PublicationControl {
  std::atomic<std::uint32_t> active_index{kNoActiveSlot};
  std::uint32_t reserved0{};
  std::atomic<std::uint64_t> publish_epoch{0};
  std::array<std::byte, 48> reserved{};
};

struct alignas(kCacheLineSize) SlotControl {
  std::atomic<std::uint64_t> guard{0};
  std::atomic<std::uint64_t> sequence{0};
  std::atomic<std::int64_t> timestamp_ns{0};
  std::array<std::byte, 40> reserved{};
};

static_assert(sizeof(PublicationControl) == kCacheLineSize);
static_assert(sizeof(SlotControl) == kCacheLineSize);
static_assert(sizeof(std::atomic<std::uint32_t>) == sizeof(std::uint32_t));
static_assert(alignof(std::atomic<std::uint32_t>) == alignof(std::uint32_t));
static_assert(sizeof(std::atomic<std::uint64_t>) == sizeof(std::uint64_t));
static_assert(alignof(std::atomic<std::uint64_t>) == alignof(std::uint64_t));
static_assert(sizeof(std::atomic<std::int64_t>) == sizeof(std::int64_t));
static_assert(alignof(std::atomic<std::int64_t>) == alignof(std::int64_t));
static_assert(std::is_trivially_copyable_v<std::atomic<std::uint32_t>>);
static_assert(std::is_trivially_copyable_v<std::atomic<std::uint64_t>>);
static_assert(std::is_trivially_copyable_v<std::atomic<std::int64_t>>);
static_assert(std::is_trivially_copyable_v<PublicationControl>);
static_assert(std::is_trivially_copyable_v<SlotControl>);
static_assert(std::is_standard_layout_v<PublicationControl>);
static_assert(std::is_standard_layout_v<SlotControl>);
static_assert(offsetof(PublicationControl, active_index) == 0);
static_assert(offsetof(PublicationControl, publish_epoch) == 8);
static_assert(offsetof(SlotControl, guard) == 0);
static_assert(offsetof(SlotControl, sequence) == 8);
static_assert(offsetof(SlotControl, timestamp_ns) == 16);

inline bool checked_add(std::size_t lhs, std::size_t rhs, std::size_t& output) noexcept {
  if (lhs > std::numeric_limits<std::size_t>::max() - rhs) {
    return false;
  }
  output = lhs + rhs;
  return true;
}

inline bool checked_multiply(std::size_t lhs, std::size_t rhs,
                             std::size_t& output) noexcept {
  if (lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
    return false;
  }
  output = lhs * rhs;
  return true;
}

inline bool checked_align_cache_line(std::size_t value, std::size_t& output) noexcept {
  std::size_t rounded{};
  if (!checked_add(value, kCacheLineSize - 1, rounded)) {
    return false;
  }
  output = rounded & ~(kCacheLineSize - 1);
  return true;
}

template <class T>
struct SnapshotLayout {
  std::size_t slot_stride{};
  std::size_t mapping_size{};
  std::size_t words_per_axis{};
};

template <class T>
Result<SnapshotLayout<T>> calculate_layout(std::uint32_t axis_count) {
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::is_standard_layout_v<T>);
  static_assert(sizeof(T) % sizeof(std::uint64_t) == 0);
  static_assert(alignof(T) <= alignof(std::uint64_t));

  if (axis_count > kRobotIoMaximumAxes) {
    return Result<SnapshotLayout<T>>::failure(
        {ErrorCode::invalid_argument, "IPC axis count must be between 0 and 12"});
  }
  std::size_t payload_size{};
  std::size_t unaligned_slot_size{};
  std::size_t slot_stride{};
  std::size_t slots_size{};
  std::size_t prefix_size{};
  std::size_t mapping_size{};
  if (!checked_multiply(axis_count, sizeof(T), payload_size) ||
      !checked_add(sizeof(SlotControl), payload_size, unaligned_slot_size) ||
      !checked_align_cache_line(unaligned_slot_size, slot_stride) ||
      !checked_multiply(2, slot_stride, slots_size) ||
      !checked_add(sizeof(IpcHeader), sizeof(PublicationControl), prefix_size) ||
      !checked_add(prefix_size, slots_size, mapping_size)) {
    return Result<SnapshotLayout<T>>::failure(
        {ErrorCode::invalid_argument, "IPC mapping size overflow"});
  }
  return Result<SnapshotLayout<T>>::success(
      {slot_stride, mapping_size, sizeof(T) / sizeof(std::uint64_t)});
}

}  // namespace detail

template <class T>
struct Snapshot {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  std::uint32_t axis_count{};
  std::array<T, kRobotIoMaximumAxes> axes{};
};

template <class T>
class SnapshotRegion {
 public:
  SnapshotRegion() = default;

  static Result<std::size_t> mapping_size(std::uint32_t axis_count) {
    auto layout = detail::calculate_layout<T>(axis_count);
    if (!layout.has_value()) {
      return Result<std::size_t>::failure(layout.error());
    }
    return Result<std::size_t>::success(layout.value().mapping_size);
  }

  static Result<SnapshotRegion> initialize(void* address, std::size_t size,
                                           std::uint32_t axis_count,
                                           std::uint32_t generation,
                                           IpcRegionKind kind) {
    auto address_check = validate_address(address);
    if (!address_check.has_value()) {
      return Result<SnapshotRegion>::failure(address_check.error());
    }
    auto layout = detail::calculate_layout<T>(axis_count);
    if (!layout.has_value() || layout.value().mapping_size != size) {
      return Result<SnapshotRegion>::failure(
          {ErrorCode::invalid_argument, "IPC mapping size does not encode a valid axis count"});
    }
    auto* base = static_cast<std::byte*>(address);
    auto* header = std::construct_at(reinterpret_cast<IpcHeader*>(base));
    header->generation = generation;
    header->axis_count = axis_count;
    header->axis_stride = sizeof(T);
    header->region_kind = static_cast<std::uint32_t>(kind);
    header->slot_stride = layout.value().slot_stride;
    header->mapping_size = layout.value().mapping_size;

    auto* publication = std::construct_at(reinterpret_cast<detail::PublicationControl*>(
        base + sizeof(IpcHeader)));
    auto region = from_layout(base, size, axis_count, header, publication, layout.value());
    for (std::uint32_t slot_index = 0; slot_index < 2; ++slot_index) {
      auto* slot = std::construct_at(region.slot(slot_index));
      (void)slot;
      auto* words = region.payload_words(slot_index);
      const auto word_count = static_cast<std::size_t>(axis_count) *
                              layout.value().words_per_axis;
      for (std::size_t word = 0; word < word_count; ++word) {
        std::construct_at(words + word, std::uint64_t{0});
      }
    }
    if (!region.atomics_are_lock_free()) {
      return Result<SnapshotRegion>::failure(
          {ErrorCode::unavailable, "process-shared IPC atomics are not lock-free"});
    }
    return Result<SnapshotRegion>::success(region);
  }

  static Result<SnapshotRegion> attach(void* address, std::size_t size,
                                       std::uint32_t expected_generation,
                                       std::uint32_t expected_axis_count,
                                       IpcRegionKind expected_kind) {
    auto address_check = validate_address(address);
    if (!address_check.has_value()) {
      return Result<SnapshotRegion>::failure(address_check.error());
    }
    auto layout = detail::calculate_layout<T>(expected_axis_count);
    if (!layout.has_value()) {
      return Result<SnapshotRegion>::failure(layout.error());
    }
    if (layout.value().mapping_size != size) {
      return Result<SnapshotRegion>::failure(
          {ErrorCode::protocol, "received IPC mapping has an unexpected size"});
    }
    auto* base = static_cast<std::byte*>(address);
    auto* header = reinterpret_cast<IpcHeader*>(base);
    auto validation = validate_header_fields(
        *header, expected_generation, expected_axis_count, sizeof(T), expected_kind,
        layout.value().mapping_size, layout.value().slot_stride);
    if (!validation.has_value()) {
      return Result<SnapshotRegion>::failure(validation.error());
    }
    auto* publication = reinterpret_cast<detail::PublicationControl*>(
        base + sizeof(IpcHeader));
    auto region = from_layout(base, size, expected_axis_count, header, publication,
                              layout.value());
    if (!region.atomics_are_lock_free()) {
      return Result<SnapshotRegion>::failure(
          {ErrorCode::unavailable, "process-shared IPC atomics are not lock-free"});
    }
    return Result<SnapshotRegion>::success(region);
  }

  Result<void> publish(std::span<const T> axes, std::uint64_t sequence,
                       std::int64_t timestamp_ns) noexcept {
    if (publication_ == nullptr || axes.size() != axis_count_) {
      // An empty diagnostic keeps the post-mapping real-time path free of
      // dynamic string storage even when the caller supplies the wrong span.
      return Result<void>::failure({ErrorCode::invalid_argument, {}});
    }

    // This is a single-writer protocol. The inactive slot is marked odd, every
    // payload word and metadata field is atomically replaced, the slot is marked
    // even, and only then is it published as active. Sequential consistency is
    // intentional: its single global order prevents a reader that still observes
    // the old even guard from observing payload stores ordered after the new odd
    // guard. This avoids both torn payloads and C++ data races without a mutex.
    const auto active = publication_->active_index.load(std::memory_order_seq_cst);
    const std::uint32_t target = active < 2 ? 1U - active : 0U;
    auto* target_slot = slot(target);
    const auto epoch = publication_->publish_epoch.fetch_add(2, std::memory_order_seq_cst) + 2;
    target_slot->guard.store(epoch | 1U, std::memory_order_seq_cst);

    auto* words = payload_words(target);
    for (std::size_t axis = 0; axis < axes.size(); ++axis) {
      std::array<std::byte, sizeof(T)> bytes{};
      std::memcpy(bytes.data(), &axes[axis], sizeof(T));
      for (std::size_t word_index = 0; word_index < words_per_axis_; ++word_index) {
        std::uint64_t word{};
        std::memcpy(&word, bytes.data() + word_index * sizeof(word), sizeof(word));
        words[axis * words_per_axis_ + word_index].store(word,
                                                         std::memory_order_seq_cst);
      }
    }
    target_slot->sequence.store(sequence, std::memory_order_seq_cst);
    target_slot->timestamp_ns.store(timestamp_ns, std::memory_order_seq_cst);
    target_slot->guard.store(epoch, std::memory_order_seq_cst);
    publication_->active_index.store(target, std::memory_order_seq_cst);
    return Result<void>::success();
  }

  Result<Snapshot<T>> read_latest() const noexcept {
    if (publication_ == nullptr) {
      return Result<Snapshot<T>>::failure({ErrorCode::unavailable, {}});
    }
    for (unsigned int attempt = 0; attempt < detail::kSnapshotReadAttempts; ++attempt) {
      // The bounded reader accepts a slot only when the two guard observations
      // match and are even. All payload words are atomic, so a concurrent second
      // publication can cause a retry but never a non-atomic data race.
      const auto active = publication_->active_index.load(std::memory_order_seq_cst);
      if (active >= 2) {
        return Result<Snapshot<T>>::failure({ErrorCode::unavailable, {}});
      }
      const auto* source_slot = slot(active);
      const auto first_guard = source_slot->guard.load(std::memory_order_seq_cst);
      if ((first_guard & 1U) != 0) {
        continue;
      }

      Snapshot<T> snapshot{};
      snapshot.axis_count = axis_count_;
      snapshot.sequence = source_slot->sequence.load(std::memory_order_seq_cst);
      snapshot.timestamp_ns = source_slot->timestamp_ns.load(std::memory_order_seq_cst);
      const auto* words = payload_words(active);
      for (std::size_t axis = 0; axis < axis_count_; ++axis) {
        std::array<std::byte, sizeof(T)> bytes{};
        for (std::size_t word_index = 0; word_index < words_per_axis_; ++word_index) {
          const auto word = words[axis * words_per_axis_ + word_index].load(
              std::memory_order_seq_cst);
          std::memcpy(bytes.data() + word_index * sizeof(word), &word, sizeof(word));
        }
        std::memcpy(&snapshot.axes[axis], bytes.data(), sizeof(T));
      }
      const auto second_guard = source_slot->guard.load(std::memory_order_seq_cst);
      if (first_guard == second_guard && (second_guard & 1U) == 0) {
        return Result<Snapshot<T>>::success(snapshot);
      }
    }
    return Result<Snapshot<T>>::failure({ErrorCode::unavailable, {}});
  }

  std::uint32_t axis_count() const noexcept { return axis_count_; }
  std::uint32_t generation() const noexcept {
    return header_ == nullptr ? 0 : header_->generation;
  }

 private:
  static Result<void> validate_address(void* address) {
    if (address == nullptr ||
        reinterpret_cast<std::uintptr_t>(address) % detail::kCacheLineSize != 0) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "IPC mapping address must be cache-line aligned"});
    }
    return Result<void>::success();
  }

  static SnapshotRegion from_layout(std::byte* base, std::size_t size,
                                    std::uint32_t axis_count, IpcHeader* header,
                                    detail::PublicationControl* publication,
                                    const detail::SnapshotLayout<T>& layout) {
    SnapshotRegion region;
    region.base_ = base;
    region.size_ = size;
    region.axis_count_ = axis_count;
    region.slot_stride_ = layout.slot_stride;
    region.words_per_axis_ = layout.words_per_axis;
    region.header_ = header;
    region.publication_ = publication;
    return region;
  }

  bool atomics_are_lock_free() const noexcept {
    if (!publication_->active_index.is_lock_free() ||
        !publication_->publish_epoch.is_lock_free()) {
      return false;
    }
    for (std::uint32_t slot_index = 0; slot_index < 2; ++slot_index) {
      const auto* current_slot = slot(slot_index);
      if (!current_slot->guard.is_lock_free() || !current_slot->sequence.is_lock_free() ||
          !current_slot->timestamp_ns.is_lock_free()) {
        return false;
      }
    }
    const auto word_count = static_cast<std::size_t>(axis_count_) * words_per_axis_;
    return word_count == 0 || payload_words(0)->is_lock_free();
  }

  detail::SlotControl* slot(std::uint32_t index) const noexcept {
    return reinterpret_cast<detail::SlotControl*>(
        base_ + sizeof(IpcHeader) + sizeof(detail::PublicationControl) +
        static_cast<std::size_t>(index) * slot_stride_);
  }

  std::atomic<std::uint64_t>* payload_words(std::uint32_t index) const noexcept {
    return reinterpret_cast<std::atomic<std::uint64_t>*>(
        reinterpret_cast<std::byte*>(slot(index)) + sizeof(detail::SlotControl));
  }

  std::byte* base_{};
  std::size_t size_{};
  std::uint32_t axis_count_{};
  std::size_t slot_stride_{};
  std::size_t words_per_axis_{};
  IpcHeader* header_{};
  detail::PublicationControl* publication_{};
};

template <class T>
Result<void> validate_header(const IpcHeader& header, std::uint32_t expected_generation,
                             std::uint32_t expected_axis_count, IpcRegionKind expected_kind,
                             std::size_t expected_mapping_size) {
  auto layout = detail::calculate_layout<T>(expected_axis_count);
  if (!layout.has_value()) {
    return Result<void>::failure(layout.error());
  }
  return validate_header_fields(header, expected_generation, expected_axis_count, sizeof(T),
                                expected_kind, expected_mapping_size,
                                layout.value().slot_stride);
}

}  // namespace policy_runtime
