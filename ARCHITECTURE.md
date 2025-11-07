# Complete Architecture Documentation
## RISC-V RV32I/RV32F Pipelined CPU Simulator

**Course**: Computer Architecture (ECGR 4181/5181)
**Assignment**: Part 4 - Pipelined CPU with RAM and Bus Arbitration

---

## Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Memory Organization](#memory-organization)
4. [Pipeline Design](#pipeline-design)
5. [Timing Model](#timing-model)
6. [Instruction Set](#instruction-set)
7. [Stack Management](#stack-management)
8. [Hazard Handling](#hazard-handling)
9. [Code Organization](#code-organization)
10. [Detailed Examples](#detailed-examples)
11. [Metrics & Analysis](#metrics--analysis)

---

## 1. Overview

This simulator models a **4-stage pipelined RISC-V CPU** executing RV32I (32-bit integer) and RV32F (single-precision floating-point) instructions with cycle-accurate timing.

### Key Features:
- ✅ 4-stage pipeline: **Fetch → Decode → Execute → Writeback**
- ✅ Dual-port memory: **I-port** (instruction fetch) + **D-port** (data access with 2-cycle latency)
- ✅ Scoreboard-based hazard detection (RAW dependencies)
- ✅ Variable execute latencies: **1 cycle** (integer) vs **5 cycles** (floating-point)
- ✅ Stack-based function calls with frame pointer
- ✅ CPI (Cycles Per Instruction) metrics

### What It Simulates:
```c
// Vector addition: C[i] = A[i] + B[i] for i = 0..255
for (int i = 0; i < 256; i++) {
    C[i] = A[i] + B[i];  // Floating-point addition
}
```

---

## 2. System Architecture

### High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                       MAIN SYSTEM                            │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                    CPU (Pipelined)                    │  │
│  │                                                        │  │
│  │  ┌────────┐  ┌─────────┐  ┌─────────┐  ┌──────────┐ │  │
│  │  │ FETCH  │→ │ DECODE  │→ │ EXECUTE │→ │WRITEBACK │ │  │
│  │  └───┬────┘  └────┬────┘  └────┬────┘  └────┬─────┘ │  │
│  │      │            │            │             │        │  │
│  │      │  ┌─────────▼────────────▼─────────────▼──┐    │  │
│  │      │  │      Register Files                    │    │  │
│  │      │  │  • 32 Integer Regs (x0-x31)           │    │  │
│  │      │  │  • 32 FP Regs (f0-f31)                │    │  │
│  │      │  │  • PC (Program Counter)               │    │  │
│  │      │  │  • SP (Stack Pointer = x2)            │    │  │
│  │      │  └───────────────────────────────────────┘    │  │
│  │      │                                                │  │
│  └──────┼────────────────────────────────────────────────┘  │
│         │                                                    │
│         │  ┌────────────────────────────────────────────┐  │
│         │  │             MEMORY (64KB)                   │  │
│         │  │                                             │  │
│         │  │  ┌──────────────────────────────────────┐  │  │
│         └──┼─→│ I-Port (Instruction Fetch)          │  │  │
│            │  │   • No latency                       │  │  │
│            │  │   • Read-only                        │  │  │
│            │  └──────────────────────────────────────┘  │  │
│            │                                             │  │
│            │  ┌──────────────────────────────────────┐  │  │
│            └─→│ D-Port (Data Access)                │  │  │
│               │   • 2-cycle latency (20 ticks)      │  │  │
│               │   • Read/Write                       │  │  │
│               │   • Request queue (up to 8)         │  │  │
│               └──────────────────────────────────────┘  │  │
│                                                           │  │
│               ┌───────────────────────────────────────┐  │  │
│               │ Memory Layout (Byte-Addressable)     │  │  │
│               │                                       │  │  │
│               │ 0x000-0x0FF: Instructions (vadd)     │  │  │
│               │ 0x100-0x1FF: Instructions (vsub)     │  │  │
│               │ 0x200-0x2FF: Stack (grows down)      │  │  │
│               │ 0x400-0x7FF: Array A (256×float32)   │  │  │
│               │ 0x800-0xBFF: Array B (256×float32)   │  │  │
│               │ 0xC00-0xFFF: Array C (results)       │  │  │
│               │ 0x1000-0x13FF: Array D (Part 2)      │  │  │
│               └───────────────────────────────────────┘  │  │
│                                                           │  │
└───────────────────────────────────────────────────────────┘
```

---

## 3. Memory Organization

### Memory Map (Detailed)

```
┌─────────────────┬──────────────┬─────────────────────────────────────┐
│ Address Range   │ Size         │ Contents                            │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x0000-0x00FF   │ 256 bytes    │ vadd program (CPU0)                 │
│                 │              │ • Prologue (stack setup)            │
│                 │              │ • Main loop (256 iterations)        │
│                 │              │ • Epilogue (cleanup & return)       │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x0100-0x01FF   │ 256 bytes    │ vsub program (CPU1, Part 2 only)    │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x0200-0x02FF   │ 256 bytes    │ STACK (grows downward)              │
│                 │              │ • 0x02FF: Top (initial SP)          │
│                 │              │ • 0x02F0: After prologue (SP-16)    │
│                 │              │ • Stores: ra, s0, local vars        │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x0300-0x03FF   │ 256 bytes    │ (unused)                            │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x0400-0x07FF   │ 1024 bytes   │ Array A (256 × 4-byte floats)       │
│                 │              │ • Initialized with random values    │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x0800-0x0BFF   │ 1024 bytes   │ Array B (256 × 4-byte floats)       │
│                 │              │ • Initialized with random values    │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x0C00-0x0FFF   │ 1024 bytes   │ Array C (vadd results: A+B)         │
│                 │              │ • Initially zeros                   │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x1000-0x13FF   │ 1024 bytes   │ Array D (vsub results: A-B)         │
│                 │              │ • Part 2 only                       │
├─────────────────┼──────────────┼─────────────────────────────────────┤
│ 0x1400-0xFFFF   │ ~59KB        │ (unused)                            │
└─────────────────┴──────────────┴─────────────────────────────────────┘
```

### Memory Access Characteristics

**I-Port (Instruction Fetch)**:
- **Latency**: 0 cycles (instant access)
- **Type**: Read-only
- **Usage**: CPU fetches instructions every cycle
- **No contention**: Separate from data accesses

**D-Port (Data Access)**:
- **Latency**: 2 CPU cycles = 20 simulation ticks
- **Type**: Read/Write
- **Request Queue**: Up to 8 pending requests
- **Usage**: Load/Store instructions (LW, SW, FLW, FSW)

**Example Timeline**:
```
Tick 0:   CPU issues LW request
Tick 10:  (waiting...)
Tick 20:  Data arrives, CPU can use it
```

---

## 4. Pipeline Design

### 4-Stage Pipeline Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        PIPELINE STAGES                           │
└──────────────────────────────────────────────────────────────────┘

Stage 1: FETCH (FE)
────────────────────────────────────────────────────────────
  Input:  PC (Program Counter)
  Action: • Read instruction from memory[PC] via I-port
          • Store in fetch_latch
  Output: next_pc = PC + 4 (default, branches override)

  Stall Conditions:
  • Decode stage is stalled (propagate backward)

  Flush Conditions:
  • Branch/Jump taken (wrong path prediction)


Stage 2: DECODE (DE)
────────────────────────────────────────────────────────────
  Input:  fetch_latch (instruction)
  Action: • Decode instruction fields (opcode, rd, rs1, rs2, imm)
          • Read register values from register file
          • Check for hazards (RAW dependencies)
          • Mark destination register as BUSY
  Output: decode_latch (decoded instruction + operand values)

  Stall Conditions:
  • Source register is BUSY (data hazard)
  • Execute stage is busy with multi-cycle operation

  Hazard Examples:
  ┌─────────────────────────────────────┐
  │ lw   a0, 0(s0)    ← loads a0       │
  │ addi a1, a0, 1    ← needs a0 STALL!│
  └─────────────────────────────────────┘


Stage 3: EXECUTE (EX)
────────────────────────────────────────────────────────────
  Input:  decode_latch (instruction + operands)
  Action: • Perform ALU/FPU operation
          • Calculate memory addresses
          • Issue memory requests (loads/stores)
          • Evaluate branch conditions
  Output: execute_latch (result + metadata)

  Multi-Cycle Operations:
  • RV32I (integer):  1 cycle  (10 ticks) - countdown = 0
  • RV32F (FP):       5 cycles (50 ticks) - countdown = 4
  • Memory access:    2 cycles (20 ticks) - mem_pending flag

  During countdown > 0:
  • Pipeline STALLS (fetch and decode cannot advance)

  During mem_pending:
  • Poll memory port each tick
  • When data ready, clear mem_pending flag


Stage 4: WRITEBACK (WB)
────────────────────────────────────────────────────────────
  Input:  execute_latch (result)
  Action: • Write result to destination register
          • Clear register BUSY flag
          • Increment retired instruction counter
  Output: (none - end of pipeline)

  Register Write Examples:
  • LW:      regs[rd] = memory_data
  • FLW:     fregs[rd] = memory_data (as float)
  • ADD:     regs[rd] = rs1 + rs2
  • FADD.S:  fregs[rd] = frs1 + frs2
  • Stores:  (no register write)
```

### Pipeline Latches (Data Structures)

```cpp
┌─────────────────────────────────────────────────────────────┐
│ FetchLatch                                                   │
│ ─────────────────────────────────────────────────────────── │
│ bool valid = false;        // Is this latch valid?          │
│ u32 pc = 0;                // PC of this instruction         │
│ u32 instruction = 0;       // 32-bit encoded instruction    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ DecodeLatch                                                  │
│ ─────────────────────────────────────────────────────────── │
│ bool valid = false;                                          │
│ u32 pc = 0;                                                  │
│ Decoded decoded;           // Decoded instruction struct     │
│ u32 rs1_val = 0;           // Value of source register 1     │
│ u32 rs2_val = 0;           // Value of source register 2     │
│ f32 frs1_val = 0.0f;       // FP source register 1           │
│ f32 frs2_val = 0.0f;       // FP source register 2           │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ ExecuteLatch                                                 │
│ ─────────────────────────────────────────────────────────── │
│ bool valid = false;                                          │
│ u32 pc = 0;                                                  │
│ Decoded decoded;                                             │
│ u32 result = 0;            // Integer result                 │
│ f32 fresult = 0.0f;        // FP result                      │
│ int execute_countdown = 0; // Cycles remaining (0-4)         │
│ bool mem_pending = false;  // Waiting for memory?            │
│ int mem_req_id = -1;       // Memory request identifier      │
└─────────────────────────────────────────────────────────────┘
```

### Pipeline State Machine

```
┌────────────────────────────────────────────────────────────┐
│                  CPU::tick() - Main Loop                   │
│ ────────────────────────────────────────────────────────── │
│                                                            │
│ 1. mem_.dport().tick()        ← Advance memory timing      │
│                                                            │
│ 2. stage_writeback()          ← Execute in REVERSE order   │
│ 3. stage_execute()            ← to avoid data races        │
│ 4. stage_decode()                                          │
│ 5. stage_fetch()                                           │
│                                                            │
│ 6. pc_ = next_pc_             ← Update PC                  │
│ 7. flush_fetch_ = false       ← Clear control signals      │
│ 8. flush_decode_ = false                                   │
│                                                            │
│ 9. stats_.ticks++             ← Update statistics          │
│ 10. if (ticks % 10 == 0)                                   │
│        stats_.cycles++                                     │
└────────────────────────────────────────────────────────────┘

Why reverse order?
• If we did Fetch→Decode→Execute→Writeback in the same tick,
  data would propagate through all stages in ONE tick (unrealistic!)
• Reverse order ensures each stage processes its OLD input,
  then writes output for NEXT tick
```

---

## 5. Timing Model

### Clock and Tick Relationship

```
┌───────────────────────────────────────────────────────────────┐
│                    TIMING HIERARCHY                           │
└───────────────────────────────────────────────────────────────┘

1 CPU Cycle = 10 Simulation Ticks
═══════════════════════════════════════════════════════════════

Tick:  0    10   20   30   40   50   60   70   80   90  100...
       ├────┼────┼────┼────┼────┼────┼────┼────┼────┼────┤
Cycle: 0    1    2    3    4    5...
       └─────────┴─────────┴─────────┴─────────┴─────────┘

All timing is driven by TICKS (not cycles)
• Pipeline advances every tick
• Memory port advances every tick
• Cycle counter increments every 10 ticks
```

### Instruction Latencies

```cpp
namespace timing {
    // Core timing constants
    inline constexpr int kTicksPerCycle = 10;

    // Execute stage latencies (in CPU cycles)
    inline constexpr int kIExecuteCycles = 1;  // RV32I: 1 cycle
    inline constexpr int kFExecuteCycles = 5;  // RV32F: 5 cycles

    // Memory latencies (in CPU cycles)
    inline constexpr int kMemCycles = 2;       // RAM: 2 cycles
}
```

**RV32I Instructions (1 cycle = 10 ticks)**:
```
Instruction    | Execute Time | Example
───────────────┼──────────────┼─────────────────────────
ADD, SUB       | 1 cycle      | add a0, a1, a2
ADDI, ANDI     | 1 cycle      | addi sp, sp, -16
SLLI, SRLI     | 1 cycle      | slli a1, a1, 2
LUI, AUIPC     | 1 cycle      | lui a0, 0x400
BEQ, BLT       | 1 cycle      | blt a1, a0, .loop
JAL, JALR      | 1 cycle      | jal ra, func
```

**RV32F Instructions (5 cycles = 50 ticks)**:
```
Instruction    | Execute Time | Example
───────────────┼──────────────┼─────────────────────────
FADD.S         | 5 cycles     | fadd.s ft0, ft0, ft1
FSUB.S         | 5 cycles     | fsub.s ft0, ft0, ft1
FMUL.S         | 5 cycles     | fmul.s ft0, ft0, ft1
FDIV.S         | 5 cycles     | fdiv.s ft0, ft0, ft1
```

**Memory Operations (1 cycle execute + 2 cycle latency = 3 total)**:
```
Instruction    | Execute | Latency | Total  | Example
───────────────┼─────────┼─────────┼────────┼─────────────────
LW             | 1 cycle | 2 cycle | 3 cyc  | lw a0, 0(sp)
SW             | 1 cycle | 2 cycle | 3 cyc  | sw a0, -16(s0)
FLW            | 1 cycle | 2 cycle | 3 cyc  | flw ft0, 0(a0)
FSW            | 1 cycle | 2 cycle | 3 cyc  | fsw ft0, 0(a0)
```

### Example: FADD.S Timing

```
┌──────────────────────────────────────────────────────────────┐
│ Instruction: fadd.s ft0, ft0, ft1                            │
│ Execute latency: 5 cycles (50 ticks)                         │
└──────────────────────────────────────────────────────────────┘

Tick    Cycle   Stage       Action
─────────────────────────────────────────────────────────────
0       0       Fetch       Read instruction from memory
10      1       Decode      Decode, read ft0, ft1 values
                            Mark ft0 as BUSY
20      2       Execute     Start FADD, countdown = 4
30      3       Execute     countdown = 3  (pipeline stalled)
40      4       Execute     countdown = 2  (pipeline stalled)
50      5       Execute     countdown = 1  (pipeline stalled)
60      6       Execute     countdown = 0  (ready for WB)
70      7       Writeback   Write ft0 = result
                            Clear ft0 BUSY
                            Retire instruction
```

---

## 6. Instruction Set

### Supported RV32I Instructions

```
┌─────────────────────────────────────────────────────────────────┐
│ ARITHMETIC & LOGICAL                                             │
├──────────────┬────────────────────────┬──────────────────────────┤
│ Instruction  │ Format                 │ Operation                │
├──────────────┼────────────────────────┼──────────────────────────┤
│ ADD          │ add rd, rs1, rs2       │ rd = rs1 + rs2           │
│ SUB          │ sub rd, rs1, rs2       │ rd = rs1 - rs2           │
│ ADDI         │ addi rd, rs1, imm      │ rd = rs1 + imm           │
│ AND          │ and rd, rs1, rs2       │ rd = rs1 & rs2           │
│ OR           │ or rd, rs1, rs2        │ rd = rs1 | rs2           │
│ XOR          │ xor rd, rs1, rs2       │ rd = rs1 ^ rs2           │
│ ANDI         │ andi rd, rs1, imm      │ rd = rs1 & imm           │
│ ORI          │ ori rd, rs1, imm       │ rd = rs1 | imm           │
│ XORI         │ xori rd, rs1, imm      │ rd = rs1 ^ imm           │
│ SLLI         │ slli rd, rs1, shamt    │ rd = rs1 << shamt        │
│ SRLI         │ srli rd, rs1, shamt    │ rd = rs1 >> shamt (log)  │
│ SRAI         │ srai rd, rs1, shamt    │ rd = rs1 >> shamt (arith)│
│ SLT          │ slt rd, rs1, rs2       │ rd = (rs1 < rs2) ? 1 : 0 │
│ SLTI         │ slti rd, rs1, imm      │ rd = (rs1 < imm) ? 1 : 0 │
│ LUI          │ lui rd, imm            │ rd = imm << 12           │
│ AUIPC        │ auipc rd, imm          │ rd = PC + (imm << 12)    │
└──────────────┴────────────────────────┴──────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ MEMORY ACCESS                                                    │
├──────────────┬────────────────────────┬──────────────────────────┤
│ LW           │ lw rd, offset(rs1)     │ rd = mem[rs1 + offset]   │
│ SW           │ sw rs2, offset(rs1)    │ mem[rs1+offset] = rs2    │
└──────────────┴────────────────────────┴──────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ CONTROL FLOW                                                     │
├──────────────┬────────────────────────┬──────────────────────────┤
│ BEQ          │ beq rs1, rs2, offset   │ if (rs1==rs2) PC+=offset │
│ BNE          │ bne rs1, rs2, offset   │ if (rs1!=rs2) PC+=offset │
│ BLT          │ blt rs1, rs2, offset   │ if (rs1<rs2)  PC+=offset │
│ BGE          │ bge rs1, rs2, offset   │ if (rs1>=rs2) PC+=offset │
│ JAL          │ jal rd, offset         │ rd=PC+4; PC+=offset      │
│ JALR         │ jalr rd, offset(rs1)   │ rd=PC+4; PC=rs1+offset   │
└──────────────┴────────────────────────┴──────────────────────────┘
```

### Supported RV32F Instructions

```
┌─────────────────────────────────────────────────────────────────┐
│ FLOATING-POINT OPERATIONS (Single Precision)                    │
├──────────────┬────────────────────────┬──────────────────────────┤
│ Instruction  │ Format                 │ Operation                │
├──────────────┼────────────────────────┼──────────────────────────┤
│ FLW          │ flw frd, offset(rs1)   │ frd = mem[rs1+offset]    │
│ FSW          │ fsw frs2, offset(rs1)  │ mem[rs1+offset] = frs2   │
│ FADD.S       │ fadd.s frd, frs1, frs2 │ frd = frs1 + frs2        │
│ FSUB.S       │ fsub.s frd, frs1, frs2 │ frd = frs1 - frs2        │
│ FMUL.S       │ fmul.s frd, frs1, frs2 │ frd = frs1 * frs2        │
│ FDIV.S       │ fdiv.s frd, frs1, frs2 │ frd = frs1 / frs2        │
└──────────────┴────────────────────────┴──────────────────────────┘

Note: All FP operations have 5-cycle execute latency
```

### Register Conventions

```
┌──────────┬─────────┬──────────────────────────────────────────┐
│ Register │ ABI Name│ Purpose                                  │
├──────────┼─────────┼──────────────────────────────────────────┤
│ x0       │ zero    │ Hardwired to 0 (always reads as zero)   │
│ x1       │ ra      │ Return Address (for function calls)     │
│ x2       │ sp      │ Stack Pointer (grows downward)          │
│ x3       │ gp      │ Global Pointer                           │
│ x4       │ tp      │ Thread Pointer                           │
│ x5-x7    │ t0-t2   │ Temporary registers (caller-saved)      │
│ x8       │ s0/fp   │ Saved register / Frame Pointer          │
│ x9       │ s1      │ Saved register (callee-saved)           │
│ x10-x11  │ a0-a1   │ Function args / return values           │
│ x12-x17  │ a2-a7   │ Function arguments                       │
│ x18-x27  │ s2-s11  │ Saved registers (callee-saved)          │
│ x28-x31  │ t3-t6   │ Temporary registers (caller-saved)      │
├──────────┼─────────┼──────────────────────────────────────────┤
│ f0-f7    │ ft0-ft7 │ FP temporary registers                   │
│ f8-f9    │ fs0-fs1 │ FP saved registers                       │
│ f10-f11  │ fa0-fa1 │ FP function args / return values         │
│ f12-f17  │ fa2-fa7 │ FP function arguments                    │
│ f18-f27  │ fs2-fs11│ FP saved registers                       │
│ f28-f31  │ ft8-ft11│ FP temporary registers                   │
└──────────┴─────────┴──────────────────────────────────────────┘
```

---

## 7. Stack Management

### Stack Overview

The stack is a critical data structure used for:
1. **Saving return addresses** (ra) during function calls
2. **Saving callee-saved registers** (s0, s1, etc.)
3. **Storing local variables** (like loop counter `i`)
4. **Creating stack frames** for nested function calls

### Stack Properties

```
┌─────────────────────────────────────────────────────────────┐
│ Stack Characteristics                                        │
├─────────────────────────────────────────────────────────────┤
│ • Base Address:   0x0300 (top of stack region)             │
│ • Size:           256 bytes (0x200 - 0x2FF)                │
│ • Growth:         DOWNWARD (decreasing addresses)           │
│ • Alignment:      4-byte aligned (word-aligned)             │
│ • Stack Pointer:  x2 (sp register)                          │
│ • Frame Pointer:  x8 (s0 register)                          │
└─────────────────────────────────────────────────────────────┘

Stack Growth:
            HIGH ADDRESS
            ↑
    0x02FF  ├─────────┐ ← Initial SP (empty stack)
            │         │
    0x02F0  ├─────────┤ ← SP after prologue (SP - 16)
            │ ra      │   (saved return address)
    0x02EC  ├─────────┤
            │ s0      │   (saved frame pointer)
    0x02E8  ├─────────┤
            │ local_1 │   (local variable)
    0x02E4  ├─────────┤
            │ local_2 │   (loop counter i)
    0x02E0  ├─────────┤ ← FP - 16 (s0 - 16) = i
            │         │
            ↓
            LOW ADDRESS
```

### Stack Frame Structure

Each function creates a **stack frame** with this layout:

```
┌─────────────────────────────────────────────────────────────┐
│                    STACK FRAME LAYOUT                        │
│                                                              │
│      Higher Addresses (0x300)                               │
│             ↑                                                │
│   ┌─────────┴──────────┐                                    │
│   │ Previous frame     │                                    │
│   ├────────────────────┤ ← Old SP (caller's SP)            │
│   │ Return address (ra)│ ← SP + 12  (saved by callee)      │
│   ├────────────────────┤                                    │
│   │ Saved s0 (old FP)  │ ← SP + 8                           │
│   ├────────────────────┤                                    │
│   │ Local var 1        │ ← SP + 4   (optional)              │
│   ├────────────────────┤                                    │
│   │ Local var 2        │ ← SP + 0   (current SP)            │
│   ├────────────────────┤ ← FP (s0) = SP + 16               │
│   │ (room for more)    │                                    │
│   └────────────────────┘                                    │
│             ↓                                                │
│      Lower Addresses (0x200)                                │
└─────────────────────────────────────────────────────────────┘
```

### Function Prologue (Stack Setup)

```assembly
# Function entry - save state and allocate stack space

1. addi sp, sp, -16        # Allocate 16 bytes on stack
                           # SP: 0x300 → 0x2F0

2. sw ra, 12(sp)           # Save return address
                           # mem[0x2F0 + 12 = 0x2FC] = ra

3. sw s0, 8(sp)            # Save old frame pointer
                           # mem[0x2F0 + 8 = 0x2F8] = s0

4. addi s0, sp, 16         # Set new frame pointer
                           # s0 = 0x2F0 + 16 = 0x300
                           # (points to base of our frame)
```

**Visual Before/After**:
```
BEFORE (at function entry):
┌────────────┬───────────┐
│ SP = 0x300 │ s0 = ???  │
└────────────┴───────────┘
Stack:
0x300: [empty] ← SP

AFTER prologue:
┌────────────┬───────────┐
│ SP = 0x2F0 │ s0 = 0x300│
└────────────┴───────────┘
Stack:
0x300: (frame base) ← s0 (FP)
0x2FC: [ra saved]
0x2F8: [old s0]
0x2F4: [unused]
0x2F0: [unused] ← SP
```

### Using the Stack for Local Variables

```assembly
# Store local variable 'i' on stack

1. addi a0, x0, 0          # a0 = 0 (initial value of i)

2. sw a0, -16(s0)          # Store at FP - 16
                           # Address: 0x300 - 16 = 0x2F0
                           # mem[0x2F0] = 0

# Later: load 'i' from stack

3. lw a0, -16(s0)          # Load from FP - 16
                           # a0 = mem[0x2F0] = 0

# Increment 'i'

4. addi a0, a0, 1          # a0 = a0 + 1 = 1

5. sw a0, -16(s0)          # Store back to stack
                           # mem[0x2F0] = 1
```

**Why use Frame Pointer (s0)?**
- **Stability**: FP doesn't change during function execution
- **Consistency**: Local variables always at same offset from FP
- **Debugging**: Makes stack traces easier
- **Nested calls**: Each function has its own frame

### Function Epilogue (Stack Cleanup)

```assembly
# Function exit - restore state and deallocate stack

1. lw a0, -12(s0)          # Load return value (optional)
                           # a0 = mem[0x300 - 12 = 0x2F4]

2. lw s0, 8(sp)            # Restore old frame pointer
                           # s0 = mem[0x2F0 + 8 = 0x2F8]

3. lw ra, 12(sp)           # Restore return address
                           # ra = mem[0x2F0 + 12 = 0x2FC]

4. addi sp, sp, 16         # Deallocate stack space
                           # SP: 0x2F0 → 0x300

5. ret                     # Return (jalr x0, 0(ra))
                           # PC = ra
```

### Complete Stack Example: vadd Loop

```assembly
# Complete function with stack usage

main:
    # === PROLOGUE ===
    addi sp, sp, -16       # Allocate stack
    sw   ra, 12(sp)        # Save return address
    sw   s0, 8(sp)         # Save frame pointer
    addi s0, sp, 16        # Set new frame pointer

    # === BODY ===
    # Initialize i = 0
    addi a0, x0, 0         # a0 = 0
    sw   a0, -16(s0)       # i = 0 (at FP-16)

.loop:
    # Load i
    lw   a0, -16(s0)       # a0 = i

    # Check if i < 256
    addi a1, x0, 255       # a1 = 255
    blt  a1, a0, .done     # if (255 < i) goto .done

    # [... vector addition code ...]

    # Increment i
    lw   a0, -16(s0)       # a0 = i
    addi a0, a0, 1         # a0 = i + 1
    sw   a0, -16(s0)       # i = i + 1
    j    .loop             # goto .loop

.done:
    # === EPILOGUE ===
    lw   a0, -12(s0)       # Load return value
    lw   s0, 8(sp)         # Restore frame pointer
    lw   ra, 12(sp)        # Restore return address
    addi sp, sp, 16        # Deallocate stack
    ret                    # Return to caller
```

**Stack Timeline**:
```
Time    SP      s0      Stack Contents               Action
─────────────────────────────────────────────────────────────
Entry   0x300   ???     [empty]                      Function called
+1      0x2F0   ???     0x2FC:[ra] 0x2F8:[old_s0]    Prologue
+2      0x2F0   0x300   (same) s0 set                Prologue done
+3      0x2F0   0x300   0x2F0:[i=0]                  Initialize i
Loop    0x2F0   0x300   0x2F0:[i=0..255]             Loop iterations
Exit    0x300   (old)   [restored]                   Epilogue done
```

---

## 8. Hazard Handling

### Types of Hazards

```
┌─────────────────────────────────────────────────────────────┐
│ 1. DATA HAZARDS (Read After Write - RAW)                    │
├─────────────────────────────────────────────────────────────┤
│ Occurs when an instruction needs data from a previous       │
│ instruction that hasn't completed yet.                      │
│                                                             │
│ Example:                                                    │
│   lw   a0, 0(s0)    ← loads a0 (takes 3 cycles)             │
│   addi a1, a0, 1    ← needs a0 → MUST STALL!                │
│                                                             │
│ Solution: Scoreboard tracks which registers are BUSY        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 2. STRUCTURAL HAZARDS                                       │
├─────────────────────────────────────────────────────────────┤
│ Occurs when multiple instructions need the same hardware    │
│ resource (e.g., memory port) simultaneously.                │
│                                                             │
│ Example:                                                    │
│   lw   a0, 0(s0)    ← using data port                       │
│   lw   a1, 4(s0)    ← also needs data port → STALL!         │
│                                                             │
│ Solution: Request queue (max 8 pending requests)            │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 3. CONTROL HAZARDS                                          │
├─────────────────────────────────────────────────────────────┤
│ Occurs when a branch/jump changes PC, making fetched        │
│ instructions from wrong path.                               │
│                                                             │
│ Example:                                                    │
│   beq a0, a1, .target  ← changes PC if taken                │
│   add a2, a3, a4       ← already fetched (WRONG!)           │
│                                                             │
│ Solution: Flush pipeline when branch taken                  │
└─────────────────────────────────────────────────────────────┘
```

### Scoreboard-Based Hazard Detection

```cpp
┌─────────────────────────────────────────────────────────────┐
│ Scoreboard Mechanism                                        │
├─────────────────────────────────────────────────────────────┤
│ Two arrays track register busy status:                      │
│                                                             │
│ std::array<bool, 32> int_busy_;   // Integer regs x0-x31    │
│ std::array<bool, 32> fp_busy_;    // FP regs f0-f31         │
│                                                             │
│ • Set to TRUE when instruction enters DECODE                │
│ • Checked by subsequent instructions                        │
│ • Cleared to FALSE when instruction completes WRITEBACK     │
└─────────────────────────────────────────────────────────────┘

Lifecycle Example:
──────────────────────────────────────────────────────────────
Cycle   Instruction     a0 busy?   Action
──────────────────────────────────────────────────────────────
1       lw a0, 0(s0)    → TRUE     Mark a0 busy (decode)
2       (executing...)   TRUE      a0 still busy
3       (waiting...)     TRUE      Waiting for memory
4       addi a1,a0,1    TRUE       Check: a0 busy? YES → STALL
5       (lw completes)  → FALSE    Clear a0 busy (writeback)
6       addi a1,a0,1    FALSE      Check: a0 busy? NO → proceed
```

### Hazard Detection Algorithm

```cpp
bool Cpu::check_hazard(const Decoded& d) {
    // Check source register 1
    if (d.rs1 != 0) {  // x0 never busy
        if (d.is_fp()) {
            if (fp_busy_[d.rs1]) return true;  // FP hazard
        } else {
            if (int_busy_[d.rs1]) return true; // Integer hazard
        }
    }

    // Check source register 2
    if (d.rs2 != 0) {
        // For stores: rs2 is data source
        if (d.kind == SW && int_busy_[d.rs2]) return true;
        if (d.kind == FSW && fp_busy_[d.rs2]) return true;

        // For R-type: rs2 is operand
        if (d.is_fp() && fp_busy_[d.rs2]) return true;
        if (!d.is_fp() && !d.is_memory() && int_busy_[d.rs2])
            return true;
    }

    return false;  // No hazard detected
}
```

### Stalling Example

```
Instruction Sequence:
──────────────────────────────────────────────────────────────
1. lw   a0, 0(s0)      # Load a0 from memory
2. addi a1, a0, 1      # Use a0 (HAZARD!)
3. add  a2, a1, a0     # Use a1 and a0

Pipeline Timeline (without forwarding):
──────────────────────────────────────────────────────────────
Tick  | FE       | DE       | EX       | WB       | Notes
──────────────────────────────────────────────────────────────
0     | lw(1)    | -        | -        | -        |
10    | addi(2)  | lw(1)    | -        | -        | Mark a0 busy
20    | add(3)   | addi(2)  | lw(1)    | -        | Hazard! Stall
30    | add(3)   | addi(2)  | lw(1)    | -        | Still stalled
40    | add(3)   | addi(2)  | lw(1)    | -        | Memory ready
50    | add(3)   | addi(2)  | -        | lw(1)    | Clear a0 busy
60    | -        | add(3)   | addi(2)  | -        | Mark a1 busy
70    | -        | -        | add(3)   | addi(2)  | Clear a1 busy
80    | -        | -        | -        | add(3)   | Complete

Total: 8 cycles for 3 instructions → CPI = 2.67
Without hazard: 5 cycles → CPI = 1.67
Stall penalty: 3 cycles
```

---

## 9. Code Organization

### File-by-File Component Mapping

```
┌───────────────────────────────────────────────────────────────┐
│ Component Layer 1: TYPE DEFINITIONS                            │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ types.hpp                                                      │
│ ├─ u8, u16, u32, u64, i32, f32 (type aliases)                │
│ └─ struct Decoded                                              │
│    ├─ raw: u32                     (original instruction)     │
│    ├─ opcode, rd, rs1, rs2, funct3, funct7                   │
│    ├─ imm: i32                     (immediate value)          │
│    ├─ enum Kind { ADD, LW, FADD_S, ... }                     │
│    └─ Helper methods: is_fp(), is_memory(), is_branch()      │
│                                                                │
│ Used by: ALL other files (foundation)                         │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│ Component Layer 2: TIMING & CONFIGURATION                      │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ timing.hpp                                                     │
│ └─ namespace timing {                                          │
│    ├─ kTicksPerCycle = 10                                     │
│    ├─ kIExecuteCycles = 1                                     │
│    ├─ kFExecuteCycles = 5                                     │
│    ├─ kMemCycles = 2                                          │
│    ├─ cycles_to_ticks(int cycles)                            │
│    └─ ticks_to_cycles(int ticks)                             │
│ }                                                              │
│                                                                │
│ Used by: cpu.cpp, memory.cpp, main_single.cpp                │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│ Component Layer 3: INSTRUCTION SET ARCHITECTURE                │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ isa.hpp / isa.cpp                                             │
│ └─ namespace isa {                                             │
│    │                                                           │
│    ├─ DECODING                                                │
│    │  └─ Decoded decode(u32 raw)                             │
│    │     • Extract fields (opcode, rs1, rs2, rd, imm)        │
│    │     • Identify instruction kind (ADD, LW, FADD.S, etc)  │
│    │     • Handle all instruction formats (R, I, S, B, U, J) │
│    │                                                           │
│    ├─ INTEGER ALU                                             │
│    │  ├─ u32 exec_add(u32 a, u32 b)                          │
│    │  ├─ u32 exec_sub(u32 a, u32 b)                          │
│    │  ├─ u32 exec_and(u32 a, u32 b)                          │
│    │  ├─ u32 exec_or(u32 a, u32 b)                           │
│    │  ├─ u32 exec_xor(u32 a, u32 b)                          │
│    │  ├─ u32 exec_sll(u32 a, u32 shamt)                      │
│    │  ├─ u32 exec_srl(u32 a, u32 shamt)                      │
│    │  ├─ u32 exec_sra(i32 a, u32 shamt)                      │
│    │  ├─ u32 exec_slt(i32 a, i32 b)                          │
│    │  └─ u32 exec_sltu(u32 a, u32 b)                         │
│    │                                                           │
│    ├─ FLOATING-POINT ALU                                      │
│    │  ├─ f32 exec_fadd_s(f32 a, f32 b)                       │
│    │  ├─ f32 exec_fsub_s(f32 a, f32 b)                       │
│    │  ├─ f32 exec_fmul_s(f32 a, f32 b)                       │
│    │  └─ f32 exec_fdiv_s(f32 a, f32 b)                       │
│    │                                                           │
│    ├─ BRANCH EVALUATION                                       │
│    │  ├─ bool eval_beq(u32 a, u32 b)                         │
│    │  ├─ bool eval_bne(u32 a, u32 b)                         │
│    │  ├─ bool eval_blt(i32 a, i32 b)                         │
│    │  ├─ bool eval_bge(i32 a, i32 b)                         │
│    │  ├─ bool eval_bltu(u32 a, u32 b)                        │
│    │  └─ bool eval_bgeu(u32 a, u32 b)                        │
│    │                                                           │
│    └─ ADDRESS CALCULATION                                     │
│       ├─ u32 calc_branch_target(u32 pc, i32 offset)          │
│       ├─ u32 calc_jump_target(u32 pc, i32 offset)            │
│       └─ u32 calc_jalr_target(u32 base, i32 offset)          │
│ }                                                              │
│                                                                │
│ Used by: cpu.cpp (for decoding and executing instructions)   │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│ Component Layer 4: MEMORY SUBSYSTEM                            │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ memory.hpp / memory.cpp                                       │
│ │                                                              │
│ ├─ struct MemReq                  (Memory Request)            │
│ │  ├─ enum Op { Read, Write }                                │
│ │  ├─ u32 addr                    (address)                   │
│ │  ├─ u32 wdata                   (write data)                │
│ │  └─ int id                      (request ID)                │
│ │                                                              │
│ ├─ struct MemRsp                  (Memory Response)           │
│ │  ├─ bool ready                  (is data ready?)           │
│ │  ├─ u32 rdata                   (read data)                 │
│ │  └─ int id                      (request ID)                │
│ │                                                              │
│ ├─ class DataRamPort             (Data Port with Latency)    │
│ │  ├─ DataRamPort(vector<u8>& mem)                           │
│ │  ├─ bool can_issue()            (queue space available?)   │
│ │  ├─ void issue(MemReq)          (submit request)           │
│ │  ├─ MemRsp poll()               (check completion)         │
│ │  ├─ void tick()                 (advance time)             │
│ │  │                                                           │
│ │  └─ Private:                                                │
│ │     ├─ struct PendingReq { MemReq req, int completion_tick }│
│ │     ├─ queue<PendingReq> pending_                          │
│ │     ├─ int current_tick_                                    │
│ │     ├─ u32 read_word(u32 addr)                             │
│ │     └─ void write_word(u32 addr, u32 data)                 │
│ │                                                              │
│ └─ class Memory                   (Main Memory)              │
│    ├─ Memory(size_t size = 64KB)                             │
│    ├─ u32 ifetch(u32 addr)        (instant instruction fetch)│
│    ├─ DataRamPort& dport()        (get data port)            │
│    ├─ u8& operator[](u32 addr)    (direct access)            │
│    ├─ void write_u32(u32 addr, u32 value)                    │
│    ├─ u32 read_u32(u32 addr)                                 │
│    ├─ void write_f32(u32 addr, f32 value)                    │
│    ├─ f32 read_f32(u32 addr)                                 │
│    │                                                           │
│    └─ Private:                                                │
│       ├─ vector<u8> data_         (byte array, 64KB)         │
│       └─ DataRamPort dport_       (data port instance)       │
│                                                                │
│ Used by: cpu.cpp (for instruction fetch and data access),    │
│          loader.cpp (for initialization)                      │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│ Component Layer 5: CPU PIPELINE (CORE)                        │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ cpu.hpp / cpu.cpp                                             │
│ │                                                              │
│ ├─ struct Stats                   (Performance Metrics)       │
│ │  ├─ u64 ticks, cycles, retired                             │
│ │  ├─ u64 loads, stores, stalls                              │
│ │  └─ double cpi()                                            │
│ │                                                              │
│ ├─ struct FetchLatch              (Fetch→Decode buffer)      │
│ │  ├─ bool valid                                              │
│ │  ├─ u32 pc                                                  │
│ │  └─ u32 instruction                                         │
│ │                                                              │
│ ├─ struct DecodeLatch             (Decode→Execute buffer)    │
│ │  ├─ bool valid                                              │
│ │  ├─ u32 pc                                                  │
│ │  ├─ Decoded decoded                                         │
│ │  ├─ u32 rs1_val, rs2_val        (int register values)      │
│ │  └─ f32 frs1_val, frs2_val      (FP register values)       │
│ │                                                              │
│ ├─ struct ExecuteLatch            (Execute→Writeback buffer) │
│ │  ├─ bool valid                                              │
│ │  ├─ u32 pc                                                  │
│ │  ├─ Decoded decoded                                         │
│ │  ├─ u32 result                  (integer result)           │
│ │  ├─ f32 fresult                 (FP result)                │
│ │  ├─ int execute_countdown       (cycles remaining)         │
│ │  ├─ bool mem_pending            (waiting for memory?)      │
│ │  └─ int mem_req_id              (memory request ID)        │
│ │                                                              │
│ └─ class Cpu                      (Main CPU)                 │
│    │                                                           │
│    ├─ PUBLIC API                                              │
│    │  ├─ Cpu(Memory& mem, int cpu_id)                        │
│    │  ├─ void reset(u32 pc0)                                 │
│    │  ├─ Stats run(u64 max_ticks)                            │
│    │  ├─ void tick()                                          │
│    │  ├─ bool halted()                                        │
│    │  ├─ Stats& stats()                                       │
│    │  ├─ u32 get_ireg(u8 reg)      (debug: read int reg)     │
│    │  ├─ f32 get_freg(u8 reg)      (debug: read FP reg)      │
│    │  └─ u32 get_pc()               (debug: read PC)          │
│    │                                                           │
│    ├─ PRIVATE STATE                                           │
│    │  ├─ Memory& mem_               (reference to memory)    │
│    │  ├─ int cpu_id_                (CPU identifier)         │
│    │  ├─ bool halted_               (is CPU halted?)         │
│    │  ├─ u32 pc_, next_pc_          (program counters)       │
│    │  ├─ array<u32, 32> regs_       (integer registers)      │
│    │  ├─ array<f32, 32> fregs_      (FP registers)           │
│    │  ├─ array<bool, 32> int_busy_  (scoreboard: int)        │
│    │  ├─ array<bool, 32> fp_busy_   (scoreboard: FP)         │
│    │  ├─ FetchLatch fetch_latch_                             │
│    │  ├─ DecodeLatch decode_latch_                           │
│    │  ├─ ExecuteLatch execute_latch_                         │
│    │  ├─ Stats stats_                                         │
│    │  ├─ bool stall_fetch_, stall_decode_                    │
│    │  └─ bool flush_fetch_, flush_decode_                    │
│    │                                                           │
│    └─ PRIVATE METHODS                                         │
│       ├─ void stage_fetch()        (Stage 1)                 │
│       ├─ void stage_decode()       (Stage 2)                 │
│       ├─ void stage_execute()      (Stage 3)                 │
│       ├─ void stage_writeback()    (Stage 4)                 │
│       ├─ bool check_hazard(Decoded&)                         │
│       ├─ void mark_busy(Decoded&)                            │
│       ├─ void clear_busy(Decoded&)                           │
│       └─ void handle_branch(u32 target)                      │
│                                                                │
│ Used by: main_single.cpp (to run the simulation)             │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│ Component Layer 6: PROGRAM LOADING                             │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ loader.hpp / loader.cpp                                       │
│ │                                                              │
│ ├─ struct MemMap                  (Memory Layout Config)      │
│ │  ├─ u32 inst_base = 0x000                                  │
│ │  ├─ u32 inst_size = 0x100                                  │
│ │  ├─ u32 stack_base = 0x200                                 │
│ │  ├─ u32 stack_size = 0x100                                 │
│ │  ├─ u32 A_base = 0x400                                     │
│ │  ├─ u32 B_base = 0x800                                     │
│ │  ├─ u32 C_base = 0xC00                                     │
│ │  ├─ u32 D_base = 0x1000                                    │
│ │  └─ int length = 256                                        │
│ │                                                              │
│ └─ namespace loader {                                          │
│    │                                                           │
│    ├─ INSTRUCTION ENCODING                                    │
│    │  ├─ u32 encode_r_type(...)    (R-format: ADD, SUB)      │
│    │  ├─ u32 encode_i_type(...)    (I-format: ADDI, LW)      │
│    │  ├─ u32 encode_s_type(...)    (S-format: SW)            │
│    │  ├─ u32 encode_b_type(...)    (B-format: BEQ, BLT)      │
│    │  ├─ u32 encode_u_type(...)    (U-format: LUI, AUIPC)    │
│    │  └─ u32 encode_j_type(...)    (J-format: JAL)           │
│    │                                                           │
│    ├─ PROGRAM LOADING                                         │
│    │  ├─ void load_program_vadd(Memory&, MemMap&)            │
│    │  │     • Encodes vadd assembly                          │
│    │  │     • Writes to memory at inst_base                  │
│    │  │                                                        │
│    │  └─ void load_program_vsub(Memory&, MemMap&)            │
│    │        • Encodes vsub assembly                          │
│    │        • Writes to memory at 0x100                      │
│    │                                                           │
│    └─ DATA INITIALIZATION                                     │
│       ├─ void init_arrays(Memory&, MemMap&, seed)            │
│       │     • Fills A and B with random floats               │
│       │     • Uses fixed seed for reproducibility            │
│       │                                                        │
│       └─ void clear_results(Memory&, MemMap&)                │
│             • Zeros out C and D arrays                        │
│ }                                                              │
│                                                                │
│ Used by: main_single.cpp (before running simulation)         │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│ Component Layer 7: VALIDATION                                  │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ checks.hpp / checks.cpp                                       │
│ └─ namespace checks {                                          │
│    │                                                           │
│    ├─ HOST REFERENCE COMPUTATION                              │
│    │  ├─ void host_vadd(f32* A, f32* B, f32* C, int n)       │
│    │  │     • Pure C++ implementation: C[i] = A[i] + B[i]    │
│    │  │     • Runs on host CPU (not simulated)               │
│    │  │                                                        │
│    │  └─ void host_vsub(f32* A, f32* B, f32* D, int n)       │
│    │        • Pure C++ implementation: D[i] = A[i] - B[i]    │
│    │                                                           │
│    ├─ FLOATING-POINT COMPARISON                               │
│    │  └─ bool approximately_equal(f32 a, f32 b, int ulp=1)   │
│    │        • Compares with ULP tolerance                     │
│    │        • Handles NaN, infinity, zeros                    │
│    │                                                           │
│    └─ VALIDATION FUNCTIONS                                    │
│       ├─ bool validate_vadd(Memory&, MemMap&, int n)         │
│       │     • Reads simulated C[] from memory                │
│       │     • Computes reference C_ref[] on host              │
│       │     • Compares element-by-element                     │
│       │     • Returns true if all match                       │
│       │                                                        │
│       └─ bool validate_vsub(Memory&, MemMap&, int n)         │
│             • Same for D[] (vsub results)                     │
│ }                                                              │
│                                                                │
│ Used by: main_single.cpp (after simulation completes)        │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│ Component Layer 8: MAIN PROGRAM                                │
├───────────────────────────────────────────────────────────────┤
│                                                                │
│ main_single.cpp                  (Part 1 Entry Point)         │
│ │                                                              │
│ └─ int main(int argc, char** argv)                            │
│    │                                                           │
│    ├─ 1. Parse command-line arguments                         │
│    │     --ticks, --len, --seed, --help                       │
│    │                                                           │
│    ├─ 2. Create memory map (MemMap)                           │
│    │                                                           │
│    ├─ 3. Initialize memory (Memory, 64KB)                     │
│    │                                                           │
│    ├─ 4. Load program                                          │
│    │     loader::load_program_vadd(mem, map)                  │
│    │                                                           │
│    ├─ 5. Initialize arrays                                    │
│    │     loader::init_arrays(mem, map, seed)                  │
│    │     loader::clear_results(mem, map)                      │
│    │                                                           │
│    ├─ 6. Create and reset CPU                                 │
│    │     Cpu cpu(mem, 0)                                      │
│    │     cpu.reset(map.inst_base)                             │
│    │                                                           │
│    ├─ 7. Run simulation                                        │
│    │     Stats s = cpu.run(max_ticks)                         │
│    │                                                           │
│    ├─ 8. Print statistics                                      │
│    │     ticks, cycles, retired, CPI, etc.                    │
│    │                                                           │
│    ├─ 9. Validate results                                      │
│    │     bool valid = checks::validate_vadd(mem, map, n)      │
│    │                                                           │
│    └─ 10. Return exit code (0=success, 1=failure)             │
│                                                                │
│ Compiles to: build/single.exe                                 │
└───────────────────────────────────────────────────────────────┘
```

---

## 10. Detailed Examples

### Example 1: Simple ADD Instruction

```
Instruction: add a2, a0, a1
Encoding: 0x00B50633
Binary: 0000000 01011 01010 000 01100 0110011
        funct7  rs2   rs1   f3  rd    opcode
```

**Cycle-by-Cycle Execution**:

```
Tick  Cycle  Stage      Action
──────────────────────────────────────────────────────────────
0     0      FE         • PC = 0x10
                        • ifetch(0x10) → 0x00B50633
                        • fetch_latch.instruction = 0x00B50633
                        • next_pc = 0x14

10    1      DE         • decode(0x00B50633) → ADD
                        • rs1=10 (a0), rs2=11 (a1), rd=12 (a2)
                        • Read regs_[10] = 5, regs_[11] = 3
                        • check_hazard() → false (no hazard)
                        • mark_busy(a2) → int_busy_[12] = true
                        • decode_latch.rs1_val = 5
                        • decode_latch.rs2_val = 3

20    2      EX         • exec_add(5, 3) = 8
                        • execute_countdown = 0 (immediate)
                        • execute_latch.result = 8

30    3      WB         • regs_[12] = 8  (write a2)
                        • clear_busy(a2) → int_busy_[12] = false
                        • stats_.retired++
                        • Done!

Result: a2 = 8
Total time: 4 cycles (40 ticks)
CPI: 4 / 1 = 4.0 (ideal CPI = 1.0, overhead from empty pipeline)
```

### Example 2: Load-Use Hazard

```
Instructions:
1. lw  a0, 0(sp)       # Load a0 from stack
2. add a1, a0, a2      # Use a0 immediately (HAZARD!)
```

**Pipeline Timeline**:

```
Tick  Cycle | FE         | DE         | EX         | WB         |
────────────────────────────────────────────────────────────────
0     0     | lw(1)      | -          | -          | -          |
10    1     | add(2)     | lw(1)      | -          | -          |
              a0.busy=T
20    2     | add(2)     | STALLED    | lw(1)      | -          |
              Hazard!      (a0 busy)    (issue mem)
30    3     | add(2)     | STALLED    | lw(1)      | -          |
              (stalled)    (a0 busy)    (wait mem)
40    4     | add(2)     | STALLED    | lw(1)      | -          |
              (stalled)    (a0 busy)    (mem ready)
50    5     | add(2)     | STALLED    | -          | lw(1)      |
              (stalled)    (a0 busy)                  a0.busy=F
60    6     | -          | add(2)     | -          | -          |
                           (no hazard)
70    7     | -          | -          | add(2)     | -          |
80    8     | -          | -          | -          | add(2)     |

Stall cycles: 4 (ticks 20-50)
Total: 9 cycles for 2 instructions
CPI: 9 / 2 = 4.5
```

### Example 3: Floating-Point Addition

```
Instruction: fadd.s ft0, ft1, ft2
```

**Timeline**:

```
Tick  Cycle  Stage      Action                            Countdown
─────────────────────────────────────────────────────────────────
0     0      FE         Fetch instruction                 -
10    1      DE         Decode, read ft1, ft2            -
                        Mark ft0 busy
20    2      EX         Start FADD, result=ft1+ft2       4
30    3      EX         (executing...)                   3
40    4      EX         (executing...)                   2
50    5      EX         (executing...)                   1
60    6      EX         (executing...)                   0
70    7      WB         Write ft0, clear busy            -
                        Retire instruction

Total: 8 cycles (70 ticks)
FP operation alone: 5 cycles
Pipeline overhead: 3 cycles
```

### Example 4: Complete Loop Iteration

```assembly
# One iteration of: C[i] = A[i] + B[i]

.loop:
    lw   a0, -16(s0)       # 1. Load i
    slli a1, a0, 2         # 2. a1 = i * 4
    lui  a2, 0x400         # 3. a2 = 0x400 (A base)
    add  a2, a2, a1        # 4. a2 = &A[i]
    flw  ft0, 0(a2)        # 5. ft0 = A[i]
    lui  a2, 0x800         # 6. a2 = 0x800 (B base)
    add  a2, a2, a1        # 7. a2 = &B[i]
    flw  ft1, 0(a2)        # 8. ft1 = B[i]
    fadd.s ft0,ft0,ft1     # 9. ft0 = A[i] + B[i] (5 cycles!)
    lui  a2, 0xC00         # 10. a2 = 0xC00 (C base)
    add  a2, a2, a1        # 11. a2 = &C[i]
    fsw  ft0, 0(a2)        # 12. Store C[i]
    addi a0, a0, 1         # 13. i++
    sw   a0, -16(s0)       # 14. Store i
    addi a1, x0, 255       # 15. a1 = 255
    blt  a0, a1, .loop     # 16. if (i < 255) repeat

Approximate cycles per iteration:
• 16 instructions
• 2 loads (LW, 2 FLW) = 3×3 = 9 cycles
• 2 stores (FSW, SW) = 2×3 = 6 cycles
• 1 FADD.S = 5 cycles
• 11 integer ops = 11×1 = 11 cycles
• Hazards + stalls ≈ 10 cycles
• Total ≈ 41 cycles per iteration

For 256 iterations:
• Total cycles ≈ 41 × 256 = 10,496 cycles
• Total instructions = 16 × 256 = 4,096 instructions
• CPI ≈ 10,496 / 4,096 ≈ 2.56
```

---

## 11. Metrics & Analysis

### Performance Metrics

```
┌─────────────────────────────────────────────────────────────┐
│ Key Performance Indicators                                   │
├─────────────────────────────────────────────────────────────┤
│ Metric           | Formula                | Meaning          │
├──────────────────┼────────────────────────┼──────────────────┤
│ CPI              | Cycles / Retired       | Avg cycles/instr │
│ IPC              | 1 / CPI                | Instr per cycle  │
│ Throughput       | Retired / Cycles       | Same as IPC      │
│ Stall Rate       | Stalls / Cycles        | % time stalled   │
│ Memory Latency   | Loads × kMemCycles     | Cycle overhead   │
│ FP Penalty       | FP_instrs × 4          | Extra FP cycles  │
└──────────────────┴────────────────────────┴──────────────────┘
```

### Expected CPI Breakdown

```
Ideal (no hazards, no multi-cycle):      CPI = 1.0
├─ Base pipeline overhead:                +0.5  (fill/drain)
├─ Memory hazards (loads/stores):         +0.8  (2-cycle latency)
├─ Data hazards (RAW dependencies):       +0.6  (scoreboard stalls)
└─ FP operations (5-cycle execute):       +0.7  (FADD/FSUB stalls)
                                          ═════
Realistic CPI for vadd:                   ≈ 3.6

Factors:
• 256 iterations × ~16 instructions = 4,096 instructions
• ~15,000 cycles total
• CPI ≈ 3.6
```

### Statistics Output Example

```
========================================
  Simulation Statistics
========================================
  Ticks:              149,230
  Cycles:             14,923
  Retired:            4,096 instructions
  Loads:              768 (256 LW + 512 FLW)
  Stores:             512 (256 FSW + 256 SW)
  Stalls:             2,341
  CPI:                3.643
  Wall time:          127 ms
========================================

✓ Validation PASSED - Results are correct!
```

### Optimization Opportunities

```
┌─────────────────────────────────────────────────────────────┐
│ 1. Add Forwarding (Bypass Network)                          │
├─────────────────────────────────────────────────────────────┤
│ Current: Stall until writeback completes                    │
│ Improvement: Forward result from EX→DE                      │
│ Benefit: Reduce data hazard stalls by ~60%                  │
│ New CPI: 3.6 → 2.8                                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 2. Out-of-Order Execution                                    │
├─────────────────────────────────────────────────────────────┤
│ Current: Strict in-order execution                          │
│ Improvement: Execute independent instructions in parallel   │
│ Benefit: Hide memory latency                                │
│ New CPI: 2.8 → 2.0                                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 3. Branch Prediction                                         │
├─────────────────────────────────────────────────────────────┤
│ Current: Always flush on branch (pessimistic)               │
│ Improvement: Predict taken for backward branches            │
│ Benefit: Reduce control hazard penalty                      │
│ New CPI: 2.0 → 1.8                                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 4. SIMD / Vector Instructions                                │
├─────────────────────────────────────────────────────────────┤
│ Current: One float operation per instruction                │
│ Improvement: Process 4 floats per instruction (SIMD)        │
│ Benefit: 4× throughput for vectorizable code                │
│ New CPI: 1.8 → 0.5 (effective)                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Summary

This architecture demonstrates fundamental concepts of modern CPU design:

✅ **Pipelining**: Multiple instructions in flight simultaneously
✅ **Hazard Detection**: Ensuring correct execution despite dependencies
✅ **Memory Hierarchy**: Separate ports and latency modeling
✅ **Variable Latency**: Different operation types take different times
✅ **Stack Management**: Proper function call conventions

The simulator provides a realistic model of how real CPUs balance **performance** (throughput) with **correctness** (hazard handling) while executing complex programs with mixed integer and floating-point operations.
