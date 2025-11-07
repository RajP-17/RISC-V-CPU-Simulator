# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **dual-CPU RV32 pipeline simulator** for a Computer Architecture course (ECGR 4181/5181). The simulator models a 4-stage pipelined CPU architecture executing RISC-V RV32I/RV32F instructions with cycle-accurate timing, RAM latency, hazard handling, and memory bus contention.

**Assignment Parts:**
- **Part 1**: Single pipelined CPU with shared RAM executing vector addition (vadd.c)
- **Part 2**: Dual-CPU system with bus arbitration executing vadd.c (CPU0) and vsub.c (CPU1) concurrently

## Build and Run Commands

This is a **C++20** project using **CMake ≥ 3.22**.

### Build
```bash
# Configure and build
cmake -S . -B build
cmake --build build -j

# Build with warnings enabled
cmake --build build -j -- -Wall -Wextra -Wpedantic
```

### Run Simulations
```bash
# Part 1: Single CPU running vadd
./build/single --ticks 0

# Part 2: Dual CPU running vadd + vsub
./build/dual --ticks 0
```

### Testing
```bash
# Run all unit tests
ctest --test-dir build --output-on-failure
```

## Architecture and Design

### Timing Model
- **1 CPU cycle = 10 simulation ticks** (all timing is tick-driven)
- **Execute stage latencies:**
  - RV32I instructions: 1 cycle (10 ticks)
  - RV32F instructions: 5 cycles (50 ticks) — **stalls required**
- **RAM data port latency:** 2 cycles (20 ticks) end-to-end per access
- Timing constants are defined in `include/timing.hpp`

### Pipeline Architecture (4 stages)
1. **Fetch**: Instruction fetch via I-port (read-only, no data contention)
2. **Decode**: Instruction decode and operand read
3. **Execute**: ALU operations with variable latency (1 or 5 cycles)
4. **Writeback**: Register write and memory store commit

**Hazard Handling:**
- **Structural hazards**: Data port busy or memory latency → pipeline stall
- **Data hazards**: RAW on integer/FP registers → scoreboard-based stalling (no forwarding by default)
- **Control hazards**: Taken branch/jump → flush younger stages and redirect PC

### Memory Architecture

**Memory Map:**
- Instructions (vadd): `0x000–0x093` (CPU0 at 0x0)
- Instructions (vsub): `0x100–0x193` (CPU1 at 0x100)
- Stack: `0x200–0x2FF`
- Array A (float32[256]): `0x400–0x7FF`
- Array B (float32[256]): `0x800–0xBFF`
- Array C (vadd result): `0xC00–0xFFF`
- Array D (vsub result): `0x1000–0x13FF`

**Memory Ports:**
- Each CPU has two logical ports:
  - **I-port**: Instruction fetch (read-only)
  - **D-port**: Data memory (read/write with 2-cycle latency)
- Part 2 adds a **Bus Arbiter** to multiplex 4 requesters (CPU0.I/D, CPU1.I/D) to shared RAM

### ISA Support (Minimum Viable)

**RV32I Base:**
- `LUI`, `AUIPC`, `ADDI`, `ADD`, `SUB`, `SLLI`
- `LW`, `SW`
- `JAL`, `JALR`
- `BEQ`, `BNE`, `BLT`

**RV32F (single-precision):**
- `FLW`, `FSW`
- `FADD.S`, `FSUB.S`

No CSR or privileged instructions required.

## Code Structure

### Core Modules (`/sim`)

**`include/` and `src/`:**
- `types.hpp`: Type aliases (u32, i32, f32) and `Decoded` struct for instruction representation
- `isa.[hpp|cpp]`: Instruction decode and execute primitives for RV32I/F
- `memory.[hpp|cpp]`: Byte-addressable RAM with latency modeling and dual-port interface
- `cpu.[hpp|cpp]`: CPU pipeline with register files, scoreboard, and tick-driven execution
- `bus.[hpp|cpp]`: Memory bus arbiter for Part 2 (multiplexes 4 ports to shared RAM)
- `loader.[hpp|cpp]`: Memory initialization (load programs, initialize arrays A/B with pseudo-random floats)
- `timing.[hpp|cpp]`: Timing constants and tick/cycle conversion utilities
- `checks.[hpp|cpp]`: Host-side reference validation for vector operations

**Entry Points:**
- `src/main_single.cpp`: Part 1 executable (single CPU, vadd)
- `src/main_dual.cpp`: Part 2 executable (dual CPU, vadd + vsub with bus contention)

**Tests (`/tests`):**
- `test_isa.cpp`: ISA decode/execute unit tests
- `test_memory.cpp`: RAM latency and port behavior tests
- `test_cpu_single.cpp`: Single-instruction and hazard tests
- `test_dual.cpp`: Bus contention and dual-CPU integration tests

### Program Files (`/programs`)
- `vadd.c`, `vsub.c`: C source code for vector operations (reference only)
- `CPU0.s`, `CPU1.s`: RISC-V assembly reference (vadd starts at 0x0, vsub at 0x100)

## Key Deliverables

1. **Assembly code** for vadd.c and vsub.c with stack pointer usage
2. **Pipelined simulator** supporting RV32I + RV32F instructions
3. **RAM component** with latency modeling and proper CPU-RAM signaling
4. **Part 1 metrics**: CPI calculation and validation of vadd results
5. **Bus arbiter** for Part 2 with contention handling
6. **Part 2 metrics**: Per-CPU CPI and validation of both vadd and vsub results

## Development Notes

### Assumptions and Constraints
- **Single-issue, in-order** pipeline
- **Aligned memory accesses only** (4-byte aligned for word/float loads/stores)
- **No forwarding** by default (scoreboard-based stalling); forwarding may be added to reduce CPI
- **Fixed-seed pseudo-random** initialization for arrays A and B (reproducible results)
- Validation uses **1 ULP tolerance** for floating-point comparison

### Common Patterns
- All memory addresses are 4-byte aligned
- Branch/jump targets are also 4-byte aligned
- Stack grows downward from 0x2FF
- Register `x0` is hardwired to zero
- Floating-point register bank is separate from integer register bank

### Testing Strategy
- **ISA tests**: Verify encoding/decoding, arithmetic edge cases, branch computation
- **Memory tests**: Confirm latency (exactly 20 ticks), port independence, request queuing
- **CPU tests**: Single-instruction retirement, hazard stalling, load-use delays
- **Dual tests**: Bus contention increases CPI, fairness verification (round-robin arbiter)

## Important Files
- [CMakeLists.txt](CMakeLists.txt): Build configuration
- [Assignment_4.md](Assignment_4.md): Full assignment specification
- [Agent_Task_Spec_CPP.md](Agent_Task_Spec_CPP.md): Detailed implementation specification for C++ simulator
- [CPU0.s](CPU0.s): Vector addition assembly (starts at address 0x0)
- [CPU1.s](CPU1.s): Vector subtraction assembly (starts at address 0x100)
