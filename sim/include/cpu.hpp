#pragma once
#include "types.hpp"
#include "memory.hpp"
#include "isa.hpp"
#include <array>
#include <optional>

// CPU statistics
struct Stats {
    u64 ticks = 0;
    u64 cycles = 0;
    u64 retired = 0;  // Successfully completed instructions
    u64 loads = 0;
    u64 stores = 0;
    u64 stalls = 0;

    double cpi() const {
        return retired > 0 ? static_cast<double>(cycles) / static_cast<double>(retired) : 0.0;
    }
};

// Pipeline stage latches
struct FetchLatch {
    bool valid = false;
    u32 pc = 0;
    u32 instruction = 0;
};

struct DecodeLatch {
    bool valid = false;
    u32 pc = 0;
    Decoded decoded;
    u32 rs1_val = 0;
    u32 rs2_val = 0;
    f32 frs1_val = 0.0f;
    f32 frs2_val = 0.0f;
};

struct ExecuteLatch {
    bool valid = false;
    u32 pc = 0;
    Decoded decoded;
    u32 result = 0;       // ALU result or memory address
    f32 fresult = 0.0f;   // FP result
    u32 store_data = 0;   // Data to store (for SW)
    f32 fstore_data = 0.0f; // FP data to store (for FSW)
    int execute_countdown = 0;  // Cycles remaining in execute
    int mem_req_id = -1;  // Memory request ID (if issued)
    bool mem_pending = false;
};

class Cpu {
public:
    explicit Cpu(Memory& mem, int cpu_id = 0);

    // Reset CPU to initial state
    void reset(u32 pc0);

    // Run until halted or max_ticks reached
    Stats run(u64 max_ticks = 0);

    // Single-step the pipeline by one tick
    void tick();

    // Check if CPU has halted
    bool halted() const { return halted_; }

    // Get current statistics
    const Stats& stats() const { return stats_; }

    // Register access (for debugging)
    u32 get_ireg(u8 reg) const { return regs_[reg]; }
    f32 get_freg(u8 reg) const { return fregs_[reg]; }
    u32 get_pc() const { return pc_; }

private:
    Memory& mem_;
    int cpu_id_;
    bool halted_ = false;

    // Program counter
    u32 pc_ = 0;
    u32 next_pc_ = 0;

    // Register files
    std::array<u32, 32> regs_;   // Integer registers (x0-x31)
    std::array<f32, 32> fregs_;  // Floating-point registers (f0-f31)

    // Scoreboard for hazard detection
    std::array<bool, 32> int_busy_;   // Integer register busy bits
    std::array<bool, 32> fp_busy_;    // FP register busy bits

    // Pipeline latches
    FetchLatch fetch_latch_;
    DecodeLatch decode_latch_;
    ExecuteLatch execute_latch_;

    // Statistics
    Stats stats_;

    // Pipeline control
    bool stall_fetch_ = false;
    bool stall_decode_ = false;
    bool flush_fetch_ = false;
    bool flush_decode_ = false;

    // Pipeline stage functions
    void stage_fetch();
    void stage_decode();
    void stage_execute();
    void stage_writeback();

    // Helper functions
    bool check_hazard(const Decoded& d);
    void mark_busy(const Decoded& d);
    void clear_busy(const Decoded& d);
    void handle_branch(u32 target);
};
