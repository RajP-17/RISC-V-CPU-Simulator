# Coding Agent Task Spec (C++) — Dual-CPU RV32 Pipeline Simulator

**Target environment:** C++20, CMake ≥ 3.22, Linux/macOS/WSL.  
**Goal:** Build a cycle/tick-based simulator for a 4-stage pipelined **RV32I/RV32F** CPU with a byte-addressable RAM, then extend to a **dual-CPU** system sharing a memory bus/arbiter. Execute the provided vector programs:
- `programs/vadd.c` (A + B → C) on **CPU0**
- `programs/vsub.c` (A − B → D) on **CPU1**

The simulator should model instruction/FP latencies, RAM latency, basic hazards, and bus contention. Produce validation and CPI metrics.

> **Inputs provided** (drop these into `programs/`):
> - `CPU0.s` — reference RV32 disassembly (vadd loop)
> - `CPU1.s` — reference RV32 disassembly (vsub loop)
> - `vadd.c` — vector add (A+B→C) in C
> - `vsub.c` — vector sub (A−B→D) in C
> - `ASSIGNMENT.md` — key requirements extracted from the brief (fill from your assignment doc if needed)

---

## 1) Deliverables

1. A C++ repo matching the **structure** below, with complete, warning-free builds (`-Wall -Wextra -Wpedantic`).
2. A **single-CPU** run (Part 1) that executes `vadd` and reports: retired instructions, cycles, **CPI**, and correctness vs a host-side reference.
3. A **dual-CPU** run (Part 2) that runs `CPU0: vadd` and `CPU1: vsub` concurrently over a shared bus, and reports CPI per CPU and overall runtime.
4. **Unit tests** (GoogleTest or Catch2) covering ISA decode/execute primitives, RAM latency behavior, hazard handling, and end-to-end results.
5. Clear **README** with build/run instructions and explanation of design/assumptions.

---

## 2) Repository Layout (C++)

```
/sim
  /include
    bus.hpp
    cpu.hpp
    isa.hpp
    loader.hpp
    memory.hpp
    timing.hpp
    checks.hpp
    types.hpp
  /src
    bus.cpp
    cpu.cpp
    isa.cpp
    loader.cpp
    memory.cpp
    timing.cpp
    checks.cpp
    main_single.cpp     // Part 1 entry: single CPU (vadd)
    main_dual.cpp       // Part 2 entry: dual CPU (vadd + vsub)
  /programs
    CPU0.s              // provided, read-only
    CPU1.s              // provided, read-only
    vadd.c              // provided, read-only
    vsub.c              // provided, read-only
  /tests
    test_isa.cpp
    test_memory.cpp
    test_cpu_single.cpp
    test_dual.cpp
  CMakeLists.txt
  README.md
  ASSIGNMENT.md         // distilled requirements you follow (copy from your brief)
```

**CMakeLists.txt (starter):**
```cmake
cmake_minimum_required(VERSION 3.22)
project(rv32_dual_cpu_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(simcore
  src/bus.cpp src/cpu.cpp src/isa.cpp src/loader.cpp src/memory.cpp src/timing.cpp src/checks.cpp
)
target_include_directories(simcore PUBLIC include)
target_compile_options(simcore PRIVATE -Wall -Wextra -Wpedantic -O2)

add_executable(single src/main_single.cpp)
target_link_libraries(single PRIVATE simcore)

add_executable(dual src/main_dual.cpp)
target_link_libraries(dual PRIVATE simcore)

# Tests (choose one framework; example with Catch2 as header-only vendored in /tests/vendor/catch2)
add_executable(tests tests/test_isa.cpp tests/test_memory.cpp tests/test_cpu_single.cpp tests/test_dual.cpp)
target_link_libraries(tests PRIVATE simcore)
```

**README.md (key commands):**
```bash
# Configure + build
cmake -S . -B build
cmake --build build -j

# Run single CPU (vadd)
./build/single --ticks 0   # 0 = run until halt

# Run dual CPU (vadd + vsub)
./build/dual --ticks 0

# Tests
ctest --test-dir build --output-on-failure
```

---

## 3) Architectural Model & Assumptions

### 3.1 Timing constants
- **1 CPU cycle = 10 sim ticks** (global tick counter drives all state machines).
- **Execute latency**: RV32I = **1 cycle**; RV32F = **5 cycles** (stall execute stage accordingly).
- **RAM latency** (data port): **2 cycles** end-to-end per access (20 ticks). Instruction fetch uses a separate I-port (no data contention in Part 1).

> If your brief specifies different numbers, keep them configurable in `timing.hpp` constants:
```cpp
namespace timing {
  inline constexpr int kTicksPerCycle = 10;
  inline constexpr int kIExecuteCycles = 1;
  inline constexpr int kFExecuteCycles = 5;
  inline constexpr int kMemCycles = 2; // data RAM
}
```

### 3.2 Pipeline
- **Stages**: `Fetch → Decode → Execute → Writeback` (stores commit in WB, loads deliver to WB).
- **Single-issue**, in-order.
- **Hazards**:
  - **Structural**: data port busy / memory latency outstanding → stall in Execute or earlier.
  - **Data**: RAW on int/FP regs → simple scoreboard (no forwarding), or add minimal forwarding if easier.
  - **Control**: on taken branch or jump, **flush** younger stages; PC redirected.
- **Ports**: two logical ports per CPU: **I** (read-only) and **D** (read/write).

### 3.3 ISA scope (minimum viable to run the programs)
- RV32I: `LUI`, `AUIPC`, `ADDI`, `ADD`, `SUB`, `SLLI`, `LW`, `SW`, `JAL`, `JALR`, `BEQ/BNE/BLT` (whichever appears in the loop form).
- RV32F (single-precision): `FLW`, `FSW`, `FADD.S`, `FSUB.S`.
- CSR/privilege not required. Only user-level semantics needed for these loops.

### 3.4 Memory map (example consistent with the provided code)
- **Instruction** image (for vadd) at `0x000–0x093` (round to 4-byte boundaries).
- **Stack** at `0x200–0x2FF`.
- **Arrays** (float32):
  - `A`: `0x0400–0x07FF` (256 elements)
  - `B`: `0x0800–0x0BFF` (256 elements)
  - `C`: `0x0C00–...` (results of vadd)
  - `D`: `0x1000–...` (results of vsub; used in Part 2)

> Keep the map **configurable** via `loader` parameters to match your assignment’s figure exactly.

---

## 4) C++ Modules (design & responsibilities)

### 4.1 `types.hpp`
```cpp
#pragma once
#include <cstdint>
#include <array>
using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using i32 = std::int32_t;
using f32 = float;
struct Decoded {
  u32 raw{};  // raw instruction
  // fields (opcode, rd, rs1, rs2, funct3/7, imm) filled by decode()
  u8 opcode{}, rd{}, rs1{}, rs2{}, funct3{}, funct7{};
  i32 imm{};
  enum class Kind { I, F, Load, Store, Branch, Jump, Lui, Auipc, Invalid } kind{Kind::Invalid};
};
```

### 4.2 `isa.[hpp|cpp]`
- **Responsibilities**: Decode RV32I/F encodings used by the programs; produce `Decoded` with micro-op kind & operands.
- **Execute primitives**:
  - Integer ALU ops (ADD/SUB/ADDI/SLLI/etc)
  - FP ALU ops (FADD.S/FSUB.S)
  - Address generation for LW/SW/FLW/FSW
  - Branch/jump condition & PC target computation
- Keep function-level latencies external (pipeline controls timing).

### 4.3 `memory.[hpp|cpp]`
- Byte-addressable RAM with **two logical banks**: **I** (read-only), **D** (read/write). (You may implement one physical array but model independent request queues per port.)
- **Latency modeling**: data requests complete after `timing::kMemCycles * timing::kTicksPerCycle` ticks.
- Interface:
```cpp
struct MemReq { u32 addr; enum class Op{ Read, Write } op; u32 wdata; int id; };
struct MemRsp { bool ready; u32 rdata; int id; };
class DataRamPort {
 public:
  bool can_issue() const;      // queue space available
  void issue(const MemReq&);   // enqueue
  MemRsp poll();               // complete when ready
};
class Memory {
 public:
  u32 ifetch(u32 addr) const;        // instruction fetch (no data contention)
  DataRamPort& dport();              // data port handle
  // backing store initialization helpers
};
```
- Implement **unaligned** accesses as not used by these programs (assert aligned).

### 4.4 `cpu.[hpp|cpp]`
- CPU owns register files and pipeline latches; interacts with `Memory`.
- **Registers**: 32×int (x0 hardwired zero), 32×float (f0..f31).
- **Pipeline latches** per stage; `tick()` advances pipeline or stalls based on:
  - Execute **countdown** (1 or 5 cycles) for integer vs FP
  - Data memory request and response readiness
  - Pending RAW via a simple **scoreboard**:
    - Mark `rd` busy at Decode; clear at Writeback.
- Public API:
```cpp
class Cpu {
 public:
  explicit Cpu(Memory& mem, int cpu_id);
  void reset(u32 pc0);
  // Run until halt or max_ticks. Returns stats.
  struct Stats { u64 ticks{}, cycles{}, retired{}, loads{}, stores{}; double cpi() const; };
  Stats run(u64 max_ticks=0);
  // For dual run: single-step external orchestrator calls tick()
  void tick();
  bool halted() const;
  const Stats& stats() const;
 private:
  // regs, fregs, pc, pipeline state
};
```

### 4.5 `bus.[hpp|cpp]`
- **Part 2** only. A simple arbiter multiplexing **four** requesters to the data RAM:
  - CPU0.I (RO), CPU0.D (RW), CPU1.I (RO), CPU1.D (RW)
- For **data** conflicts on the same physical bank, grant 1 requester per tick (round-robin or fixed priority) and queue others.
- Provide a façade so each CPU’s data port looks like its own `DataRamPort`, while under the hood requests are arbitrated.

### 4.6 `loader.[hpp|cpp]`
- Helpers to populate RAM regions:
  - Programs (small instruction blobs representing the compiled loops).
  - Arrays `A`, `B` with deterministic **fp32** pseudo-random values (fixed seed).
  - Clear `C`/`D` regions.
- Make the memory map configurable via a struct:
```cpp
struct MemMap {
  u32 inst_base, inst_size;
  u32 stack_base, stack_size;
  u32 A_base, B_base, C_base, D_base;
  int length; // vector length (e.g., 256)
};
```

### 4.7 `timing.[hpp|cpp]`
- Global tick keeper and helpers to convert between ticks and cycles.
- Optional: a simple **event queue** for memory completions.

### 4.8 `checks.[hpp|cpp]`
- Host-side reference compute for C/D:
```cpp
void host_vadd(const float* A, const float* B, float* C, int n);
void host_vsub(const float* A, const float* B, float* D, int n);
bool approximately_equal(float a, float b, int ulp=1);
```
- Validation routine: compare pipeline output buffers vs reference. Fail fast with index on first mismatch.

---

## 5) Executables

### 5.1 `main_single.cpp`
- Build memory, load `MemMap` for Part 1 (A/B/C regions only).
- Initialize `Cpu` with starting PC (instruction base).
- Run until halt (or max ticks from CLI), then:
  - Print retired instructions, total cycles, **CPI**.
  - Validate `C` equals host `A+B` (bitwise or within 1 ULP).

### 5.2 `main_dual.cpp`
- Build memory and **Bus**.
- Two `Cpu` instances: CPU0 (vadd), CPU1 (vsub).
- Drive both CPUs by calling `tick()` on each per global tick; the `Bus` services memory requests.
- On completion, print per-CPU stats and overall ticks; validate `C` (`A+B`) and `D` (`A−B`).

---

## 6) Testing (examples)

- **ISA**: encoding/decoding round-trips; arithmetic corner cases; branch target & sign-extension.
- **Memory**: a read/write completes after exactly `kMemCycles * kTicksPerCycle`; overlapping requests stall correctly.
- **CPU single**: a fabricated 3-instruction program retires in expected cycles without hazards; a load-use hazard stalls as expected.
- **Dual**: synthetic conflict test where both CPUs hammer the same bank shows increased CPI; validate fairness (RR arbiter).

```cpp
// Example: memory latency test idea
TEST(Memory, DataLatency) {
  Memory mem;
  auto& d = mem.dport();
  ASSERT_TRUE(d.can_issue());
  d.issue({.addr=0x400, .op=MemReq::Op::Read, .wdata=0, .id=1});
  for (int t=0; t<timing::kMemCycles*timing::kTicksPerCycle-1; ++t) {
    auto rsp = d.poll();
    EXPECT_FALSE(rsp.ready);
  }
  auto rsp = d.poll();
  EXPECT_TRUE(rsp.ready);
}
```

---

## 7) Acceptance Checklist

- [ ] Compiles with C++20; no warnings with `-Wall -Wextra -Wpedantic`.
- [ ] All tests pass locally via `ctest`.
- [ ] Part 1 prints retired, cycles, **CPI**, and passes `C == A+B` (1 ULP tolerance ok).
- [ ] Part 2 prints per-CPU CPI and total ticks; passes `C == A+B` and `D == A−B`.
- [ ] Clear explanation of **hazard policy** (no-forwarding vs minimal forwarding) in README.
- [ ] Memory map and timing constants match the assignment brief (configurable in code).

---

## 8) Notes for the Agent (VS Code / Claude / Copilot)

- Prefer **clean, testable** C++ (small pure functions). Avoid premature templates; keep types explicit.
- Document any assumption not stated in the brief (e.g., branch resolution stage, exact flush penalty).
- Keep constants in one place (`timing.hpp`). Provide CLI flags: `--ticks`, `--len`, `--seed`.
- If instruction images from `CPU0.s` / `CPU1.s` don’t assemble directly, implement a minimal “synthetic loop” that mirrors the same effective sequence of: load A[i], load B[i], FADD/FSUB, store C[i]/D[i], i++, branch until `i < N`.
- Ensure **alignment** (4-byte) for word loads/stores; assert otherwise.

---

## 9) Stretch Goals (optional)

- Add simple **forwarding** from EX/WB to Decode for common RAW cases to reduce CPI.
- Model a unified RAM with bank conflicts; parameterize bus policy (RR vs fixed priority).
- Add a tiny disassembler to print retired instruction mnemonics while debugging.
- CSV export of per-instruction retire ticks for profiling.

---

## 10) Quick Stubs (to bootstrap)

```cpp
// src/main_single.cpp
#include "cpu.hpp"
#include "memory.hpp"
#include "loader.hpp"
#include "checks.hpp"
#include "timing.hpp"
#include <iostream>
int main(int argc, char** argv) {
  int n = 256; unsigned seed = 1;
  Memory mem;
  MemMap map{/*inst*/0x000, 0x100, /*stack*/0x200, 0x100,
             /*A*/0x400, /*B*/0x800, /*C*/0x0C00, /*D*/0x1000, n};
  load_program_vadd(mem, map);   // provide this helper
  init_arrays(mem, map, seed);   // fill A,B; zero C
  Cpu cpu(mem, /*id=*/0); cpu.reset(map.inst_base);
  auto s = cpu.run(/*ticks=*/0);
  bool ok = validate_vadd(mem, map, n);
  std::cout << "retired=" << s.retired << " cycles=" << s.cycles
            << " CPI=" << s.cpi() << " valid=" << ok << "\n";
  return ok ? 0 : 1;
}
```

---

### End of Spec
