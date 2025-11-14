#pragma once

#include "memory.hpp"
#include "bus.hpp"

// Wrapper around VirtualDataPort that prevents double-ticking
// The CPU will call tick() on this, but we want the tick to happen
// only in the BusArbiter, not directly on the VirtualDataPort
class VirtualDataPortWrapper {
public:
    explicit VirtualDataPortWrapper(VirtualDataPort& vport) : vport_(vport) {}

    bool can_issue() const { return vport_.can_issue(); }
    void issue(const MemReq& req) { vport_.issue(req); }
    MemRsp poll() { return vport_.poll(); }
    void tick() { /* NO-OP: ticking happens in BusArbiter */ }

private:
    VirtualDataPort& vport_;
};

// Adapter class that makes a BusArbiter's virtual port look like a Memory object
// Inherits from Memory to be compatible with CPU constructor
// Overrides ifetch and dport to delegate to the real memory and virtual port

class MemoryAdapter : public Memory {
public:
    MemoryAdapter(Memory& real_mem, VirtualDataPort& vport)
        : Memory(0)  // Create with size 0 (data_ won't be used)
        , real_mem_(real_mem)
        , vport_(vport)
        , wrapper_(vport)
    {}

    // Override instruction fetch to use real memory
    u32 ifetch(u32 addr) const override {
        return real_mem_.ifetch(addr);
    }

    // Override data port to return the wrapper
    // HACK: VirtualDataPortWrapper has the same interface as DataRamPort
    // (can_issue, issue, poll, tick) so we reinterpret_cast it
    DataRamPort& dport() override {
        return reinterpret_cast<DataRamPort&>(wrapper_);
    }

    // Override size to avoid accessing unused data_
    size_t size() const override {
        return real_mem_.size();
    }

private:
    Memory& real_mem_;
    VirtualDataPort& vport_;
    VirtualDataPortWrapper wrapper_;
};
