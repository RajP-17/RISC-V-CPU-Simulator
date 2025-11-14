#include "bus.hpp"
#include <cassert>

// ========== VirtualDataPort Implementation ==========

bool VirtualDataPort::can_issue() const {
    // Allow queueing up to 8 requests locally
    return pending_.size() < 8;
}

void VirtualDataPort::issue(const MemReq& req) {
    assert(can_issue() && "Virtual data port queue full!");
    pending_.push(req);
}

MemRsp VirtualDataPort::poll() {
    if (completed_.empty()) {
        return {false, 0, -1};
    }

    MemRsp rsp = completed_.front();
    completed_.pop();
    return rsp;
}

// ========== BusArbiter Implementation ==========

BusArbiter::BusArbiter(Memory& mem)
    : mem_(mem)
    , cpu0_dport_(0)
    , cpu1_dport_(1)
{
}

void BusArbiter::tick() {
    DataRamPort& real_dport = mem_.dport();

    // First, tick the real RAM port
    real_dport.tick();

    // Poll for completed requests from RAM
    MemRsp rsp = real_dport.poll();
    if (rsp.ready) {
        // Route response back to the correct CPU based on pending_cpu_ids_
        if (!pending_cpu_ids_.empty()) {
            int cpu_id = pending_cpu_ids_.front();
            pending_cpu_ids_.pop();

            // Deliver to the appropriate virtual port
            if (cpu_id == 0) {
                cpu0_dport_.completed_.push(rsp);
            } else {
                cpu1_dport_.completed_.push(rsp);
            }
        }
    }

    // Try to forward one request from virtual ports to real RAM
    // Round-robin between CPU0 and CPU1
    VirtualDataPort* ports[2] = {&cpu0_dport_, &cpu1_dport_};

    bool forwarded = false;

    // Try priority CPU first
    if (!ports[priority_]->pending_.empty() && real_dport.can_issue()) {
        MemReq req = ports[priority_]->pending_.front();
        ports[priority_]->pending_.pop();
        real_dport.issue(req);
        pending_cpu_ids_.push(priority_);
        forwarded = true;
        priority_ = 1 - priority_;  // Switch priority
    }
    // Try other CPU if priority didn't have a request
    else if (!ports[1 - priority_]->pending_.empty() && real_dport.can_issue()) {
        MemReq req = ports[1 - priority_]->pending_.front();
        ports[1 - priority_]->pending_.pop();
        real_dport.issue(req);
        pending_cpu_ids_.push(1 - priority_);
        forwarded = true;
        // Don't switch priority since we used the non-priority CPU
    }
}
