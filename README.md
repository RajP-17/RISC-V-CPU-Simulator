# RISC-V Pipelined CPU Simulator

A cycle-accurate simulator for a 4-stage pipelined RISC-V (RV32I/RV32F) CPU with RAM latency modeling.

## Assignment 4 - Computer Architecture

- **Part 1**: Single pipelined CPU executing vector addition (vadd.c)
- **Part 2**: Dual-CPU system with memory bus arbitration (vadd + vsub)

## Building

### Prerequisites
- C++20 compatible compiler (GCC 10+, Clang 12+, MSVC 2019+)
- CMake 3.22 or higher

### Build Commands

```bash
# Configure
cmake -S . -B build

# Build
cmake --build build -j

# The executable will be in build/single (or build/Debug/single.exe on Windows)
```

### Windows-specific (if using MSVC)
```bash
cmake -S . -B build
cmake --build build --config Release -j
```

## Running

### Part 1: Single CPU (vadd)

```bash
# Run with default settings (256-element vectors)
./build/single

# Run with custom parameters
./build/single --len 128 --seed 42

# See all options
./build/single --help
```

### Command-line Options

- `--ticks N` : Run for exactly N simulation ticks (0 = run until halt, default)
- `--len N` : Vector length (default: 256)
- `--seed N` : Random seed for array initialization (default: 1)
- `--help` : Show help message

## Architecture

### Timing Model
- **1 CPU cycle = 10 simulation ticks**
- **RV32I instructions**: 1 cycle execute latency
- **RV32F instructions**: 5 cycles execute latency
- **RAM data port**: 2 cycles latency (20 ticks)
- **Instruction fetch**: Separate port, no data contention

### Pipeline Stages
1. **Fetch**: Read instruction from I-port
2. **Decode**: Decode instruction, read registers, check hazards
3. **Execute**: ALU/FPU operations, memory address generation
4. **Writeback**: Write results to registers, commit stores

### Hazard Handling
- **Data hazards**: Scoreboard-based RAW detection (no forwarding)
- **Structural hazards**: Stall on memory port busy
- **Control hazards**: Flush pipeline on branches/jumps

### Memory Map
```
0x000 - 0x0FF : Instructions (vadd)
0x100 - 0x1FF : Instructions (vsub, Part 2 only)
0x200 - 0x2FF : Stack
0x400 - 0x7FF : Array A (float32[256])
0x800 - 0xBFF : Array B (float32[256])
0xC00 - 0xFFF : Array C (vadd results)
0x1000-0x13FF : Array D (vsub results, Part 2 only)
```

## Output

The simulator prints:
- Configuration (timing parameters, vector length)
- Execution statistics (ticks, cycles, retired instructions)
- **CPI (Cycles Per Instruction)** - main metric for performance
- Validation result (pass/fail)

Example output:
```
========================================
  Simulation Statistics
========================================
  Ticks:              125430
  Cycles:             12543
  Retired:            3856 instructions
  Loads:              512
  Stores:             256
  Stalls:             1240
  CPI:                3.253
  Wall time:          45 ms
========================================

✓ Validation PASSED - Results are correct!
```

## Implementation Files

### Core Modules (`sim/`)
- `types.hpp`: Type definitions and decoded instruction structure
- `timing.hpp`: Timing constants (ticks/cycle, latencies)
- `isa.[hpp|cpp]`: Instruction decode and execute primitives
- `memory.[hpp|cpp]`: Byte-addressable RAM with latency modeling
- `cpu.[hpp|cpp]`: 4-stage pipelined CPU with hazard detection
- `loader.[hpp|cpp]`: Memory initialization (programs and data)
- `checks.[hpp|cpp]`: Result validation against host reference
- `main_single.cpp`: Part 1 entry point (single CPU)

## Key Features

- **Cycle-accurate simulation**: Models all pipeline stages, execute latencies, and memory delays
- **Scoreboard-based hazard detection**: Prevents WAR/RAW hazards without forwarding
- **Memory latency modeling**: Queue-based data port with configurable latency
- **FP32 support**: IEEE 754 single-precision floating-point operations
- **Validation**: Automatic comparison against host-computed reference results

## Deliverables Addressed

✅ Part 1.1: Assembly code for vadd.c (embedded in loader)
✅ Part 1.2: Pipelined simulator with RV32I + RV32F support
✅ Part 1.3: RAM with latency modeling and CPU-RAM signals
✅ Part 1.4a: CPI calculation
✅ Part 1.4b: Validation against reference implementation

## Troubleshooting

### Compilation Errors
- Ensure you have C++20 support (`-std=c++20` or `/std:c++20`)
- Check CMake version: `cmake --version` (need 3.22+)

### Assertion Failures
- **Unaligned memory access**: All loads/stores must be 4-byte aligned
- **Memory out of bounds**: Check that array sizes fit within memory map

### Incorrect Results
- Check that timing constants in `timing.hpp` match assignment spec
- Verify memory map addresses match your program's expectations
- Use `--seed` to get reproducible random data for debugging

## References

- [Assignment_4.md](Assignment_4.md): Full assignment specification
- [CLAUDE.md](CLAUDE.md): Architecture overview for AI assistants
- [Agent_Task_Spec_CPP.md](Agent_Task_Spec_CPP.md): Detailed implementation spec
