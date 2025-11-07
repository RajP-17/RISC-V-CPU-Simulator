#include "memory.hpp"
#include <cstring>
#include <cassert>

// ========== DataRamPort Implementation ==========

DataRamPort::DataRamPort(std::vector<u8>& mem_backing)
    : mem_(mem_backing) {}

bool DataRamPort::can_issue() const {
    return pending_.size() < kMaxPending;
}

void DataRamPort::issue(const MemReq& req) {
    assert(can_issue() && "Data port queue full!");

    PendingReq pr;
    pr.req = req;
    // Request completes after kMemCycles * kTicksPerCycle ticks
    pr.completion_tick = current_tick_ + timing::cycles_to_ticks(timing::kMemCycles);

    pending_.push(pr);
}

MemRsp DataRamPort::poll() {
    if (pending_.empty()) {
        return {false, 0, -1};
    }

    auto& front = pending_.front();
    if (current_tick_ >= front.completion_tick) {
        // Request has completed
        MemRsp rsp;
        rsp.ready = true;
        rsp.id = front.req.id;

        if (front.req.op == MemReq::Op::Read) {
            rsp.rdata = read_word(front.req.addr);
        } else {
            write_word(front.req.addr, front.req.wdata);
            rsp.rdata = 0;
        }

        pending_.pop();
        return rsp;
    }

    return {false, 0, -1};
}

void DataRamPort::tick() {
    current_tick_++;
}

u32 DataRamPort::read_word(u32 addr) const {
    assert((addr & 0x3) == 0 && "Unaligned memory access!");
    assert(addr + 3 < mem_.size() && "Memory access out of bounds!");

    u32 value;
    std::memcpy(&value, &mem_[addr], sizeof(u32));
    return value;
}

void DataRamPort::write_word(u32 addr, u32 data) {
    assert((addr & 0x3) == 0 && "Unaligned memory access!");
    assert(addr + 3 < mem_.size() && "Memory access out of bounds!");

    std::memcpy(&mem_[addr], &data, sizeof(u32));
}

// ========== Memory Implementation ==========

Memory::Memory(size_t size)
    : data_(size, 0), dport_(data_) {}

u32 Memory::ifetch(u32 addr) const {
    assert((addr & 0x3) == 0 && "Unaligned instruction fetch!");
    if (addr + 3 >= data_.size()) {
        return 0x00000013;  // Return NOP if out of bounds
    }

    u32 value;
    std::memcpy(&value, &data_[addr], sizeof(u32));
    return value;
}

DataRamPort& Memory::dport() {
    return dport_;
}

u8& Memory::operator[](u32 addr) {
    assert(addr < data_.size() && "Memory access out of bounds!");
    return data_[addr];
}

const u8& Memory::operator[](u32 addr) const {
    assert(addr < data_.size() && "Memory access out of bounds!");
    return data_[addr];
}

void Memory::write_u32(u32 addr, u32 value) {
    assert((addr & 0x3) == 0 && "Unaligned memory write!");
    assert(addr + 3 < data_.size() && "Memory write out of bounds!");
    std::memcpy(&data_[addr], &value, sizeof(u32));
}

u32 Memory::read_u32(u32 addr) const {
    assert((addr & 0x3) == 0 && "Unaligned memory read!");
    assert(addr + 3 < data_.size() && "Memory read out of bounds!");
    u32 value;
    std::memcpy(&value, const_cast<u8*>(&data_[addr]), sizeof(u32));
    return value;
}

void Memory::write_f32(u32 addr, f32 value) {
    assert((addr & 0x3) == 0 && "Unaligned memory write!");
    assert(addr + 3 < data_.size() && "Memory write out of bounds!");
    std::memcpy(&data_[addr], &value, sizeof(f32));
}

f32 Memory::read_f32(u32 addr) const {
    assert((addr & 0x3) == 0 && "Unaligned memory read!");
    assert(addr + 3 < data_.size() && "Memory read out of bounds!");
    f32 value;
    std::memcpy(&value, const_cast<u8*>(&data_[addr]), sizeof(f32));
    return value;
}
