#pragma once
#include "types.hpp"
#include "timing.hpp"
#include <vector>
#include <queue>
#include <optional>

// Memory request structure
struct MemReq {
    enum class Op { Read, Write } op;
    u32 addr;
    u32 wdata;  // Write data (for writes)
    int id;     // Request ID for tracking
};

// Memory response structure
struct MemRsp {
    bool ready;
    u32 rdata;
    int id;
};

// Data RAM port with latency modeling
class DataRamPort {
public:
    explicit DataRamPort(std::vector<u8>& mem_backing);

    // Check if we can issue a new request
    bool can_issue() const;

    // Issue a memory request (enqueued)
    void issue(const MemReq& req);

    // Poll for completed requests
    MemRsp poll();

    // Advance time by one tick
    void tick();

    // Reset the port state (for sequential simulations)
    void reset();

private:
    struct PendingReq {
        MemReq req;
        int completion_tick;  // Tick when this request completes
    };

    std::vector<u8>& mem_;
    std::queue<PendingReq> pending_;
    int current_tick_ = 0;
    static constexpr int kMaxPending = 8;  // Max outstanding requests

    // Helper to access memory (handles 4-byte aligned reads/writes)
    u32 read_word(u32 addr) const;
    void write_word(u32 addr, u32 data);
};

// Main memory class
class Memory {
public:
    explicit Memory(size_t size = 64 * 1024);  // Default 64KB
    virtual ~Memory() = default;  // Make polymorphic

    // Instruction fetch (no latency, separate port)
    virtual u32 ifetch(u32 addr) const;

    // Data port access
    virtual DataRamPort& dport();

    // Direct access for initialization (bypasses timing)
    u8& operator[](u32 addr);
    const u8& operator[](u32 addr) const;

    // Write helpers for initialization
    void write_u32(u32 addr, u32 value);
    u32 read_u32(u32 addr) const;
    void write_f32(u32 addr, f32 value);
    f32 read_f32(u32 addr) const;

    virtual size_t size() const { return data_.size(); }

protected:
    std::vector<u8> data_;
    DataRamPort dport_;
};
