#pragma once
/// @file mpsc_ring_buffer.hpp
/// @brief Lock-free, bounded, multi-producer single-consumer ring buffer.
///
/// Producers (the Pose/Axis/Buttons subscriber callbacks, each potentially
/// running on a different i2w-internal thread) call TryPush() to hand off a
/// fixed-size record without ever taking a lock and without ever blocking
/// on disk I/O. A single consumer (the Recorder's dedicated writer thread)
/// calls TryPop() to drain records and persist them to disk.
///
/// Design notes:
///  - Producers claim a unique slot via an atomic fetch_add (wait-free --
///    no CAS loop, no spinning on contention).
///  - Each slot has its own "ready" flag with acquire/release ordering, so
///    the consumer knows when a slot's payload is safe to read, and a
///    producer knows when a slot has been fully drained and can be reused.
///  - If the consumer falls behind and a producer's claimed slot still has
///    undrained data in it, the push is rejected (overflow) rather than
///    blocking or overwriting -- callers must count/handle drops via the
///    boolean return value. This mirrors the existing
///    i2w::OverflowPolicy::DropOldest behavior used on the subscription
///    side, applied consistently on the recording side too.

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "wire_format.hpp"

namespace logger_ring
{

/// @brief Plain-data record popped out of the ring buffer by the consumer.
/// Deliberately holds no atomics -- safe to copy/return by value.
struct PoppedRecord final
{
    std::uint16_t topic_id{0};
    std::uint64_t stamp_ns{0};
    std::uint64_t seq{0};
    std::uint32_t payload_len{0};
    std::uint8_t payload[logger_wire::kMaxPayloadBytes]{};
};

namespace detail
{

/// @brief One fixed-size slot in the ring buffer. Not exposed outside this
/// header -- producers/consumer interact only via TryPush()/TryPop().
struct Slot final
{
    std::atomic<bool> ready{false};
    std::uint16_t topic_id{0};
    std::uint64_t stamp_ns{0};
    std::uint64_t seq{0};
    std::uint32_t payload_len{0};
    std::uint8_t payload[logger_wire::kMaxPayloadBytes]{};
};

} // namespace detail

/// @brief Bounded lock-free MPSC ring buffer.
/// @tparam Capacity Number of slots. Must be a power of two (enables cheap
///         index-masking instead of a modulo division on every push/pop).
template <std::size_t Capacity>
class MpscRingBuffer final
{
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

public:
    /// @brief Attempt to push one record. Wait-free for the calling producer
    /// thread -- never blocks, never touches disk.
    /// @param topic_id    Topic this record belongs to (see TopicId).
    /// @param stamp_ns    Original publish timestamp (sample.header.stamp_ns).
    /// @param seq         Sequence number from the publisher.
    /// @param data        Pointer to the raw POD struct payload.
    /// @param len         Payload length in bytes; must be <= kMaxPayloadBytes.
    /// @return false if the ring buffer is full (writer thread too slow) or
    ///         @p len exceeds the slot capacity -- caller should count this
    ///         as a dropped sample.
    bool TryPush(std::uint16_t topic_id, std::uint64_t stamp_ns, std::uint64_t seq,
                 const void* data, std::uint32_t len) noexcept
    {
        if (len > logger_wire::kMaxPayloadBytes)
        {
            return false; // payload too large for a slot -- misconfiguration
        }

        const std::uint64_t idx = write_index_.fetch_add(1, std::memory_order_relaxed);
        detail::Slot& slot = slots_[idx & (Capacity - 1)];

        // If this slot's previous occupant hasn't been drained by the
        // consumer yet, the ring is full for this slot -- reject rather
        // than overwrite (which would corrupt data the consumer is reading)
        // or block (which would defeat the purpose of decoupling).
        if (slot.ready.load(std::memory_order_acquire))
        {
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        slot.topic_id = topic_id;
        slot.stamp_ns = stamp_ns;
        slot.seq = seq;
        slot.payload_len = len;
        std::memcpy(slot.payload, data, len);

        // Release: publishes all the writes above so the consumer's
        // subsequent acquire-load is guaranteed to see them.
        slot.ready.store(true, std::memory_order_release);
        return true;
    }

    /// @brief Attempt to pop one record. Must only ever be called from the
    /// single dedicated consumer (writer) thread.
    /// @param out Filled in on success.
    /// @return false if no record is currently available.
    bool TryPop(PoppedRecord& out) noexcept
    {
        detail::Slot& slot = slots_[read_index_ & (Capacity - 1)];
        if (!slot.ready.load(std::memory_order_acquire))
        {
            return false;
        }

        out.topic_id = slot.topic_id;
        out.stamp_ns = slot.stamp_ns;
        out.seq = slot.seq;
        out.payload_len = slot.payload_len;
        std::memcpy(out.payload, slot.payload, slot.payload_len);

        // Release: signals to producers that this slot is free to reuse.
        slot.ready.store(false, std::memory_order_release);
        ++read_index_;
        return true;
    }

    /// @brief Number of records dropped so far due to a full ring buffer.
    /// Surface this as a metric/log line -- a nonzero, growing count means
    /// the writer thread (or disk) can't keep up with producer rate.
    std::uint64_t DroppedCount() const noexcept
    {
        return dropped_count_.load(std::memory_order_relaxed);
    }

private:
    std::array<detail::Slot, Capacity> slots_{};
    std::atomic<std::uint64_t> write_index_{0};
    std::uint64_t read_index_{0}; // only ever touched by the single consumer thread
    std::atomic<std::uint64_t> dropped_count_{0};
};

} // namespace logger_ring