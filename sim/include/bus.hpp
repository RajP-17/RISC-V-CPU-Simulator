#pragma once

#include "memory.hpp"
#include "types.hpp"
#include <queue>

// Simplified virtual data port - just wraps the real DataRamPort
// and tags requests with CPU ID
class VirtualDataPort {
public:
    VirtualDataPort(int cpu_id) : cpu_id_(cpu_id) {}

    bool can_issue() const;
    void issue(const MemReq& req);
    MemRsp poll();
    void tick() { /* No-op - ticking happens in BusArbiter */ }

    int cpu_id() const { return cpu_id_; }

private:
    friend class BusArbiter;
    int cpu_id_;
    std::queue<MemReq> pending_;    // Requests waiting to be sent to RAM
    std::queue<MemRsp> completed_;  // Responses from RAM
};

// Simplified bus arbiter - shares single DataRamPort between two CPUs
// Uses round-robin to forward one request per tick from virtual ports to real port
class BusArbiter {
public:
    explicit BusArbiter(Memory& mem);

    // Get access to underlying memory
    Memory& memory() { return mem_; }

    // Access to virtual ports
    VirtualDataPort& cpu0_dport() { return cpu0_dport_; }
    VirtualDataPort& cpu1_dport() { return cpu1_dport_; }

    // Tick the arbiter
    void tick();

private:
    Memory& mem_;
    VirtualDataPort cpu0_dport_;
    VirtualDataPort cpu1_dport_;
    int priority_ = 0;  // Round-robin state

    // Track which CPU each pending RAM request belongs to
    std::queue<int> pending_cpu_ids_;
};
