#pragma once

namespace timing {
    // Core timing constants (per assignment spec)
    inline constexpr int kTicksPerCycle = 10;

    // Execute stage latencies (in CPU cycles)
    inline constexpr int kIExecuteCycles = 1;   // RV32I instructions
    inline constexpr int kFExecuteCycles = 5;   // RV32F instructions

    // Memory latencies (in CPU cycles)
    inline constexpr int kMemCycles = 2;        // RAM read/write latency

    // Helper functions
    inline int cycles_to_ticks(int cycles) {
        return cycles * kTicksPerCycle;
    }

    inline int ticks_to_cycles(int ticks) {
        return (ticks + kTicksPerCycle - 1) / kTicksPerCycle;
    }
}
