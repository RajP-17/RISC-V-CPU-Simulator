# Complete Code Walkthrough Guide
## How the CPU and Simulator Work - Step by Step

---

## Table of Contents

1. [Initialization and Setup](#initialization-and-setup)
2. [Memory System Deep Dive](#memory-system-deep-dive)
3. [CPU Pipeline Execution](#cpu-pipeline-execution)
4. [Instruction Encoding and Loading](#instruction-encoding-and-loading)
5. [Complete Execution Flow Example](#complete-execution-flow-example)
6. [Bus Arbitration in Detail](#bus-arbitration-in-detail)
7. [Key Data Structures](#key-data-structures)
8. [Common Code Patterns](#common-code-patterns)

---

## Initialization and Setup

### Main Entry Point: `main_dual_simple.cpp`

Let's trace through what happens when you run `./dual_simple.exe`:

```cpp
int main(int argc, char** argv) {
    // 1. Create memory (64KB byte-addressable RAM)
    Memory mem(64 * 1024);

    // 2. Define memory map
    MemMap map;
    map.inst_base = 0x000;
    map.inst_size = 0x100;
    map.stack_base = 0x200;
    map.stack_size = 0x100;
    map.A_base = 0x400;      // Array A starts here
    map.B_base = 0x800;      // Array B starts here
    map.C_base = 0xC00;      // Result C (vadd)
    map.D_base = 0x1000;     // Result D (vsub)
    map.length = 256;        // 256 elements per array
```

**What's happening:**
1. Allocate a 64KB chunk of memory (like real RAM)
2. Define where everything lives in that memory
3. Arrays are float[256], so each takes 256 × 4 bytes = 1024 bytes

### Loading Programs

```cpp
    // 3. Load programs into memory
    loader::load_program_vadd(mem, map);  // Writes to 0x000-0x0FF
    loader::load_program_vsub(mem, map);  // Writes to 0x100-0x1FF

    // 4. Initialize arrays with random floats
    loader::init_arrays(mem, map, 1);  // seed=1 for reproducibility

    // 5. Clear result arrays
    loader::clear_results(mem, map);
```

**What's happening:**
1. Encode vadd assembly into binary instructions
2. Write those instructions to memory starting at 0x000
3. Do same for vsub starting at 0x100
4. Fill A and B arrays with random floats
5. Zero out C and D arrays (results)

### Creating and Running CPUs

```cpp
    // 6. Create CPU 0
    Cpu cpu0(mem, 0);     // CPU ID=0, reference to memory
    cpu0.reset(0x000);     // Set PC=0x000 (start of vadd)

    // 7. Run CPU 0 until halt
    Stats stats0 = cpu0.run(0);  // Run with no tick limit

    // 8. Reset memory port state!
    mem.dport().reset();   // CRITICAL: Clear tick counter

    // 9. Create and run CPU 1
    Cpu cpu1(mem, 1);
    cpu1.reset(0x100);     // Set PC=0x100 (start of vsub)
    Stats stats1 = cpu1.run(0);
```

**What's happening:**
1. Create CPU object with reference to shared memory
2. Initialize CPU state (PC, registers, pipeline)
3. Run CPU until it executes `ret` instruction
4. Reset memory state (tick counter, queues)
5. Repeat for second CPU

### Validation

```cpp
    // 10. Check results
    bool vadd_ok = checks::validate_vadd(mem, map, map.length);
    bool vsub_ok = checks::validate_vsub(mem, map, map.length);

    return (vadd_ok && vsub_ok) ? 0 : 1;
}
```

---

## Memory System Deep Dive

### Memory Class Structure

**File:** `sim/include/memory.hpp`

```cpp
class Memory {
private:
    std::vector<u8> data_;      // Raw byte array (64KB)
    DataRamPort dport_;         // Data port with latency

public:
    // Instruction fetch (instant, no latency)
    u32 ifetch(u32 addr) const {
        return read_u32(addr);  // Direct read
    }

    // Data port access (has latency)
    DataRamPort& dport() {
        return dport_;
    }
};
```

### DataRamPort: The Latency Model

**File:** `sim/src/memory.cpp`

```cpp
class DataRamPort {
private:
    std::vector<u8>& mem_;           // Reference to actual memory
    std::queue<PendingReq> pending_; // Queue of requests
    int current_tick_ = 0;           // Simulation time

    struct PendingReq {
        MemReq req;
        int completion_tick;  // When this request finishes
    };
```

**Key idea:** Requests don't complete immediately. They're queued and finish after a delay.

#### Issuing a Request

```cpp
void DataRamPort::issue(const MemReq& req) {
    // Calculate when this request will complete
    int completion_tick = current_tick_ + 20;  // 20 ticks = 2 cycles

    // Queue it up
    PendingReq pr;
    pr.req = req;
    pr.completion_tick = completion_tick;
    pending_.push(pr);
}
```

**Example timeline:**
```
Tick 0:  CPU issues load from 0x400
         → completion_tick = 0 + 20 = 20
         → Request queued
Tick 1:  CPU polls → not ready (current_tick=1 < 20)
Tick 2:  CPU polls → not ready (current_tick=2 < 20)
...
Tick 20: CPU polls → READY! (current_tick=20 >= 20)
         → Data returned
```

#### Polling for Completion

```cpp
MemRsp DataRamPort::poll() {
    if (pending_.empty()) {
        return {false, 0, -1};  // No requests
    }

    PendingReq& front = pending_.front();

    // Check if request is done
    if (current_tick_ >= front.completion_tick) {
        // Perform the actual memory access NOW
        u32 data = 0;
        if (front.req.op == MemReq::Op::Read) {
            data = read_word(front.req.addr);
        } else {
            write_word(front.req.addr, front.req.wdata);
        }

        pending_.pop();
        return {true, data, front.req.id};  // Ready!
    }

    return {false, 0, -1};  // Not ready yet
}
```

#### Advancing Time

```cpp
void DataRamPort::tick() {
    current_tick_++;  // Increment every simulator tick
}
```

**This is called from CPU's tick() method to advance simulation time.**

---

## CPU Pipeline Execution

### CPU Class Structure

**File:** `sim/include/cpu.hpp`

```cpp
class Cpu {
private:
    // Configuration
    Memory& mem_;
    int cpu_id_;

    // Architectural state
    u32 pc_;
    u32 next_pc_;
    std::array<u32, 32> int_regs_;    // x0-x31
    std::array<f32, 32> fp_regs_;     // f0-f31

    // Pipeline registers (latches)
    FetchLatch fetch_latch_;
    DecodeLatch decode_latch_;
    ExecuteLatch execute_latch_;

    // Hazard detection
    std::array<bool, 32> int_busy_;   // Scoreboard for int regs
    std::array<bool, 32> fp_busy_;    // Scoreboard for FP regs

    // Control signals
    bool stall_fetch_;
    bool stall_decode_;
    bool flush_fetch_;
    bool flush_decode_;

    // Statistics
    Stats stats_;
    bool halted_;
```

### The Tick Function: Heart of the Simulator

**File:** `sim/src/cpu.cpp`

```cpp
void Cpu::tick() {
    // Clear control signals
    stall_fetch_ = false;
    stall_decode_ = false;
    flush_fetch_ = false;
    flush_decode_ = false;

    // Execute pipeline stages in REVERSE order
    // (to avoid using "new" values from same tick)
    stage_writeback();   // 4. WB stage
    stage_execute();     // 3. EX stage
    stage_decode();      // 2. ID stage
    stage_fetch();       // 1. IF stage

    // Update PC
    pc_ = next_pc_;

    // Advance time
    stats_.ticks++;
    stats_.cycles = stats_.ticks / 10;  // 10 ticks per cycle

    // Tick memory
    mem_.dport().tick();
}
```

**Why reverse order?**
- If we did Fetch→Decode→Execute→WB, Fetch might write to `fetch_latch_` and Decode might read it **in the same tick**
- That would make it seem like the instruction moved through 2 stages in 1 tick!
- Reverse order ensures we use the "old" values (from previous tick)

### Stage 1: FETCH

```cpp
void Cpu::stage_fetch() {
    if (halted_) return;
    if (stall_fetch_) return;      // Hazard: don't fetch
    if (flush_fetch_) return;      // Branch taken: flush

    // Check if fetch latch is already occupied
    if (fetch_latch_.valid) return;  // Can't fetch, latch full

    // Fetch instruction from memory
    u32 instr = mem_.ifetch(pc_);

    // Check for halt instruction (RET = 0x00008067)
    if (instr == 0x00008067) {
        halted_ = true;
        return;
    }

    // Latch the instruction
    fetch_latch_.valid = true;
    fetch_latch_.pc = pc_;
    fetch_latch_.instruction = instr;

    // Update PC (default: sequential)
    next_pc_ = pc_ + 4;
}
```

**What's in `fetch_latch_` after fetching from PC=0x000:**
```
valid: true
pc: 0x000
instruction: 0xff010113  (addi sp, sp, -16)
```

### Stage 2: DECODE

```cpp
void Cpu::stage_decode() {
    if (stall_decode_) return;     // Hazard: don't decode
    if (flush_decode_) return;     // Branch: flush

    // Can only decode if there's something to decode
    if (!fetch_latch_.valid) return;

    // Can't decode if execute stage is full
    if (decode_latch_.valid) return;

    // Decode the instruction
    Decoded d = isa::decode(fetch_latch_.instruction);

    // Check scoreboard for RAW hazards
    bool hazard = false;
    if (d.rs1 != 0 && int_busy_[d.rs1]) hazard = true;
    if (d.rs2 != 0 && int_busy_[d.rs2]) hazard = true;
    if (d.frs1 != 0 && fp_busy_[d.frs1]) hazard = true;
    if (d.frs2 != 0 && fp_busy_[d.frs2]) hazard = true;

    if (hazard) {
        // Stall! Don't advance
        stall_fetch_ = true;
        return;
    }

    // Read source registers
    u32 rs1_val = int_regs_[d.rs1];
    u32 rs2_val = int_regs_[d.rs2];
    f32 frs1_val = fp_regs_[d.frs1];
    f32 frs2_val = fp_regs_[d.frs2];

    // Mark destination as busy (scoreboard)
    if (d.rd != 0 && d.writes_int) {
        int_busy_[d.rd] = true;
    }
    if (d.writes_fp) {
        fp_busy_[d.frd] = true;
    }

    // Latch decoded instruction + operands
    decode_latch_.valid = true;
    decode_latch_.pc = fetch_latch_.pc;
    decode_latch_.decoded = d;
    decode_latch_.rs1_val = rs1_val;
    decode_latch_.rs2_val = rs2_val;
    decode_latch_.frs1_val = frs1_val;
    decode_latch_.frs2_val = frs2_val;

    // Clear fetch latch (instruction consumed)
    fetch_latch_.valid = false;
}
```

**Example: Decoding `addi sp, sp, -16`**
```
Instruction: 0xff010113
Decoded:
  kind: ADDI
  rd: 2 (sp)
  rs1: 2 (sp)
  imm: -16
  writes_int: true

Scoreboard check: is x2 busy? No → proceed
Read operands: rs1_val = int_regs_[2] (stack pointer value)
Mark x2 busy: int_busy_[2] = true
```

### Stage 3: EXECUTE

This is the most complex stage. Let's break it down:

```cpp
void Cpu::stage_execute() {
    // Part 1: Handle ongoing execution (countdown)
    if (execute_latch_.valid) {
        if (execute_latch_.execute_countdown > 0) {
            execute_latch_.execute_countdown--;

            if (execute_latch_.execute_countdown > 0) {
                // Still executing - stall pipeline
                stall_decode_ = true;
                stall_fetch_ = true;
                return;
            }
        }

        // Part 2: Handle memory operations
        if (execute_latch_.mem_pending) {
            MemRsp rsp = mem_.dport().poll();

            if (!rsp.ready) {
                // Memory not ready - stall
                stall_decode_ = true;
                stall_fetch_ = true;
                return;
            }

            // Memory response ready!
            if (execute_latch_.decoded.is_load) {
                execute_latch_.result = rsp.rdata;
            }
            execute_latch_.mem_pending = false;
        }

        // Execution complete - move to writeback
        // (will be consumed by WB stage in same tick)
        return;
    }

    // Part 3: Start new execution
    if (!decode_latch_.valid) return;

    // Consume decode latch
    Decoded d = decode_latch_.decoded;
    decode_latch_.valid = false;

    // Execute based on instruction type
    u32 result = 0;
    f32 fresult = 0.0f;
    int countdown = 0;
    bool mem_pending = false;

    switch (d.kind) {
        case Decoded::Kind::ADD:
            result = decode_latch_.rs1_val + decode_latch_.rs2_val;
            countdown = 10;  // 1 cycle = 10 ticks
            break;

        case Decoded::Kind::FADD:
            fresult = decode_latch_.frs1_val + decode_latch_.frs2_val;
            countdown = 50;  // 5 cycles = 50 ticks
            break;

        case Decoded::Kind::LW:
            // Calculate address
            result = decode_latch_.rs1_val + d.imm;

            // Issue memory request
            MemReq req;
            req.op = MemReq::Op::Read;
            req.addr = result;
            req.id = stats_.retired;

            if (!mem_.dport().can_issue()) {
                // Memory port busy - retry next tick
                decode_latch_.valid = true;  // Put instruction back
                stall_fetch_ = true;
                return;
            }

            mem_.dport().issue(req);
            mem_pending = true;
            countdown = 20;  // 2 cycles = 20 ticks (minimum)
            break;

        // ... (other instruction types)
    }

    // Latch execution results
    execute_latch_.valid = true;
    execute_latch_.decoded = d;
    execute_latch_.result = result;
    execute_latch_.fresult = fresult;
    execute_latch_.execute_countdown = countdown;
    execute_latch_.mem_pending = mem_pending;
}
```

**Execute Timeline Example (FADD.S):**
```
Tick 100: FADD enters execute
          → fresult = 3.14 + 2.71 = 5.85
          → countdown = 50
          → execute_latch_.valid = true
Tick 101: countdown = 49, stall pipeline
Tick 102: countdown = 48, stall pipeline
...
Tick 149: countdown = 1, stall pipeline
Tick 150: countdown = 0, execution complete!
          → Move to writeback
```

### Stage 4: WRITEBACK

```cpp
void Cpu::stage_writeback() {
    if (!execute_latch_.valid) return;

    Decoded d = execute_latch_.decoded;

    // Write integer result
    if (d.writes_int && d.rd != 0) {
        int_regs_[d.rd] = execute_latch_.result;
        int_busy_[d.rd] = false;  // Clear scoreboard
    }

    // Write FP result
    if (d.writes_fp) {
        fp_regs_[d.frd] = execute_latch_.fresult;
        fp_busy_[d.frd] = false;  // Clear scoreboard
    }

    // Commit store (already issued in execute)
    // (nothing to do - store is already in memory queue)

    // Instruction retired!
    stats_.retired++;

    // Clear execute latch
    execute_latch_.valid = false;
}
```

**After WB of `addi sp, sp, -16`:**
```
int_regs_[2] = old_sp - 16  (new stack pointer)
int_busy_[2] = false        (x2 available again)
stats_.retired = 1
execute_latch_.valid = false
```

---

## Instruction Encoding and Loading

### Encoding Functions

**File:** `sim/src/isa.cpp`

All RISC-V instructions are 32-bit. Different formats pack fields differently:

#### R-Type (Register-Register)

```
Format: funct7[7] | rs2[5] | rs1[5] | funct3[3] | rd[5] | opcode[7]
Example: ADD x10, x11, x12

  31    25|24  20|19  15|14  12|11   7|6     0
  0000000 | 01100| 01011| 000  | 01010| 0110011
  funct7  |  rs2 |  rs1 |funct3|  rd  | opcode
```

```cpp
u32 encode_r_type(u8 opcode, u8 rd, u8 funct3, u8 rs1, u8 rs2, u8 funct7) {
    u32 inst = 0;
    inst |= (opcode & 0x7F);           // Bits 0-6
    inst |= (rd & 0x1F) << 7;          // Bits 7-11
    inst |= (funct3 & 0x7) << 12;      // Bits 12-14
    inst |= (rs1 & 0x1F) << 15;        // Bits 15-19
    inst |= (rs2 & 0x1F) << 20;        // Bits 20-24
    inst |= (funct7 & 0x7F) << 25;     // Bits 25-31
    return inst;
}
```

#### I-Type (Immediate)

```
Format: imm[12] | rs1[5] | funct3[3] | rd[5] | opcode[7]
Example: ADDI x10, x11, -16

  31        20|19  15|14  12|11   7|6     0
  111111110000| 01011| 000  | 01010| 0010011
     imm      |  rs1 |funct3|  rd  | opcode
```

```cpp
u32 encode_i_type(u8 opcode, u8 rd, u8 funct3, u8 rs1, i32 imm) {
    u32 inst = 0;
    inst |= (opcode & 0x7F);
    inst |= (rd & 0x1F) << 7;
    inst |= (funct3 & 0x7) << 12;
    inst |= (rs1 & 0x1F) << 15;
    inst |= (imm & 0xFFF) << 20;  // 12-bit immediate, sign-extended
    return inst;
}
```

#### J-Type (Jump)

```
Format: imm[20|10:1|11|19:12] | rd[5] | opcode[7]
Example: JAL x0, -92  (jump back to loop)

Immediate is SCRAMBLED for historical reasons!
```

```cpp
u32 encode_j_type(u8 opcode, u8 rd, i32 imm) {
    u32 inst = 0;
    inst |= (opcode & 0x7F);
    inst |= (rd & 0x1F) << 7;

    // Immediate encoding (scrambled)
    u32 imm_bits = imm & 0x1FFFFE;  // 21 bits, bit 0 always 0

    inst |= (imm_bits & 0x7FE) << 20;        // imm[10:1] → bits 30:21
    inst |= ((imm_bits >> 11) & 0x1) << 20;  // imm[11] → bit 20
    inst |= ((imm_bits >> 12) & 0xFF) << 12; // imm[19:12] → bits 19:12
    inst |= ((imm_bits >> 20) & 0x1) << 31;  // imm[20] → bit 31

    return inst;
}
```

### Loading vadd Program

**File:** `sim/src/loader.cpp`

```cpp
void load_program_vadd(Memory& mem, const MemMap& map) {
    u32 addr = 0x000;  // Start address
    std::vector<u32> program;

    // Prologue
    program.push_back(encode_i_type(0x13, 2, 0x0, 2, -16));
    // Decodes to: addi sp, sp, -16
    // 0x13 = ADDI opcode
    // rd=2 (sp), funct3=0x0, rs1=2 (sp), imm=-16

    program.push_back(encode_s_type(0x23, 0x2, 2, 1, 12));
    // sw ra, 12(sp)

    program.push_back(encode_s_type(0x23, 0x2, 2, 8, 8));
    // sw s0, 8(sp)

    program.push_back(encode_i_type(0x13, 8, 0x0, 2, 16));
    // addi s0, sp, 16  (setup frame pointer)

    // Initialize loop counter i=0
    program.push_back(encode_i_type(0x13, 10, 0x0, 0, 0));
    // addi a0, zero, 0  (a0 = 0)

    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -12));
    // sw a0, -12(s0)  (return value = 0)

    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -16));
    // sw a0, -16(s0)  (i = 0)

    // Loop starts here (index 7)
    u32 loop_check_start = program.size();

    program.push_back(encode_j_type(0x6F, 0, 4));
    // j .LBB0_1  (jump to loop condition)

    // Loop condition check
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -16));
    // lw a0, -16(s0)  (load i)

    program.push_back(encode_i_type(0x13, 11, 0x0, 0, 255));
    // addi a1, zero, 255  (a1 = 255)

    u32 blt_pos = program.size();
    program.push_back(0);  // Placeholder - will fix later

    program.push_back(encode_j_type(0x6F, 0, 4));
    // j .LBB0_2  (enter loop body)

    // Loop body: Load A[i]
    u32 A_hi = map.A_base & 0xFFFFF000;     // 0x000
    i32 A_lo = map.A_base & 0xFFF;          // 0x400
    if (A_lo & 0x800) A_hi += 0x1000;       // Adjust: no adjustment needed

    program.push_back(encode_u_type(0x37, 10, A_hi));
    // lui a0, 0x0  (a0 = 0x00000000)

    program.push_back(encode_i_type(0x13, 10, 0x0, 10, A_lo));
    // addi a0, a0, 0x400  (a0 = 0x400)

    program.push_back(encode_i_type(0x03, 11, 0x2, 8, -16));
    // lw a1, -16(s0)  (a1 = i)

    program.push_back(encode_i_type(0x13, 11, 0x1, 11, 2));
    // slli a1, a1, 2  (a1 = i * 4)

    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00));
    // add a0, a0, a1  (a0 = A_base + i*4)

    program.push_back(encode_i_type(0x07, 0, 0x2, 10, 0));
    // flw ft0, 0(a0)  (ft0 = A[i])
    // 0x07 = FLW opcode

    // Load B[i] (similar)
    u32 B_hi = 0x000;
    i32 B_lo = 0x800;
    // ... (adjust for sign extension)
    B_hi = 0x1000;
    B_lo = -0x800;  // Sign-extended

    program.push_back(encode_u_type(0x37, 10, B_hi));
    // lui a0, 0x1  (a0 = 0x00001000)

    program.push_back(encode_i_type(0x13, 10, 0x0, 10, B_lo));
    // addi a0, a0, -0x800  (a0 = 0x800)

    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00));
    // add a0, a0, a1  (a0 = B_base + i*4)

    program.push_back(encode_i_type(0x07, 1, 0x2, 10, 0));
    // flw ft1, 0(a0)  (ft1 = B[i])

    // C[i] = A[i] + B[i]
    program.push_back(encode_r_type(0x53, 0, 0x0, 0, 1, 0x00));
    // fadd.s ft0, ft0, ft1
    // 0x53 = FP opcode, funct7=0x00 for FADD

    // Store C[i] (similar to loads)
    // ... address calculation for C_base

    program.push_back(encode_s_type(0x27, 0x2, 10, 0, 0));
    // fsw ft0, 0(a0)  (C[i] = ft0)
    // 0x27 = FSW opcode

    // Increment i
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -16));
    // lw a0, -16(s0)  (a0 = i)

    program.push_back(encode_i_type(0x13, 10, 0x0, 10, 1));
    // addi a0, a0, 1  (a0 = i + 1)

    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -16));
    // sw a0, -16(s0)  (i = i + 1)

    // Jump back to loop check
    u32 jump_from = program.size();
    u32 jump_target = loop_check_start + 1;
    i32 jump_offset = (static_cast<i32>(jump_target) -
                       static_cast<i32>(jump_from)) * 4;
    program.push_back(encode_j_type(0x6F, 0, jump_offset));
    // j .LBB0_1  (back to loop check)

    // Epilogue
    u32 epilogue_start = program.size();

    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -12));
    // lw a0, -12(s0)  (load return value)

    program.push_back(encode_i_type(0x03, 8, 0x2, 2, 8));
    // lw s0, 8(sp)  (restore s0)

    program.push_back(encode_i_type(0x03, 1, 0x2, 2, 12));
    // lw ra, 12(sp)  (restore return address)

    program.push_back(encode_i_type(0x13, 2, 0x0, 2, 16));
    // addi sp, sp, 16  (restore stack pointer)

    program.push_back(0x00008067);
    // ret  (jalr x0, 0(x1))

    // Fix the BLT offset
    i32 blt_offset = (epilogue_start - blt_pos) * 4;
    program[blt_pos] = encode_b_type(0x63, 0x4, 11, 10, blt_offset);
    // blt a1, a0, .LBB0_4
    // If 255 < i, jump to epilogue

    // Write all instructions to memory
    for (size_t i = 0; i < program.size(); ++i) {
        mem.write_u32(addr + i * 4, program[i]);
    }
}
```

---

## Complete Execution Flow Example

Let's trace a single instruction through the entire pipeline:

### Instruction: `fadd.s ft0, ft0, ft1`

Assume:
- PC = 0x058
- ft0 = 3.14
- ft1 = 2.71
- Encoded as: 0x08100053

#### Tick 100: FETCH

```cpp
stage_fetch():
    pc_ = 0x058
    instr = mem_.ifetch(0x058) = 0x08100053

    fetch_latch_.valid = true
    fetch_latch_.pc = 0x058
    fetch_latch_.instruction = 0x08100053

    next_pc_ = 0x05C  (PC + 4)
```

**Pipeline State:**
```
IF: [fadd ft0,ft0,ft1]  PC=0x058
ID: [empty]
EX: [empty]
WB: [empty]
```

#### Tick 101: DECODE

```cpp
stage_decode():
    // Consume fetch latch
    d = isa::decode(0x08100053)

    // Decoded:
    d.kind = FADD
    d.frd = 0 (ft0)
    d.frs1 = 0 (ft0)
    d.frs2 = 1 (ft1)
    d.writes_fp = true

    // Check scoreboard
    fp_busy_[0] = false  ✓
    fp_busy_[1] = false  ✓
    No hazard!

    // Read operands
    frs1_val = fp_regs_[0] = 3.14
    frs2_val = fp_regs_[1] = 2.71

    // Mark destination busy
    fp_busy_[0] = true

    // Latch
    decode_latch_.valid = true
    decode_latch_.decoded = d
    decode_latch_.frs1_val = 3.14
    decode_latch_.frs2_val = 2.71

    fetch_latch_.valid = false  (consumed)
```

**Pipeline State:**
```
IF: [next instruction]
ID: [fadd ft0,ft0,ft1]  operands: 3.14, 2.71
EX: [empty]
WB: [empty]
```

#### Tick 102: EXECUTE (Start)

```cpp
stage_execute():
    // Consume decode latch
    d = decode_latch_.decoded
    decode_latch_.valid = false

    // Execute FADD
    fresult = 3.14 + 2.71 = 5.85
    countdown = 50  (5 cycles = 50 ticks)

    // Latch
    execute_latch_.valid = true
    execute_latch_.decoded = d
    execute_latch_.fresult = 5.85
    execute_latch_.execute_countdown = 50
```

**Pipeline State:**
```
IF: [...]
ID: [...]
EX: [fadd ft0,ft0,ft1]  result=5.85, countdown=50
WB: [empty]
```

#### Ticks 103-151: EXECUTE (Countdown)

```cpp
stage_execute():
    execute_latch_.execute_countdown--
    // 49, 48, 47, ..., 1

    if (execute_countdown > 0) {
        stall_decode_ = true
        stall_fetch_ = true
        return  // Pipeline stalled!
    }
```

**Pipeline State (stalled):**
```
IF: [stalled - can't fetch]
ID: [stalled - can't decode]
EX: [fadd ft0,ft0,ft1]  countdown=49...1
WB: [empty]
```

#### Tick 152: WRITEBACK

```cpp
stage_execute():
    execute_countdown = 0
    // Execution complete, ready for WB

stage_writeback():
    d = execute_latch_.decoded

    // Write result
    fp_regs_[0] = 5.85

    // Clear scoreboard
    fp_busy_[0] = false

    // Retire instruction
    stats_.retired++

    execute_latch_.valid = false
```

**Final State:**
```
fp_regs_[0] = 5.85  (ft0 updated)
fp_busy_[0] = false (ft0 available)
stats_.retired = N+1
Pipeline: all stages can advance now
```

**Total Time:** 52 ticks (Fetch + Decode + Execute(50) + WB) ≈ 5.2 cycles

---

## Bus Arbitration in Detail

### Sequential Mode (dual_simple)

No actual bus arbiter needed - just memory port reset:

```cpp
// Run CPU0
Cpu cpu0(mem, 0);
cpu0.run(0);
// CPU0 finishes, mem.dport().current_tick_ = 51870

// CRITICAL: Reset
mem.dport().reset();
// Now mem.dport().current_tick_ = 0

// Run CPU1
Cpu cpu1(mem, 1);
cpu1.run(0);
// CPU1 starts fresh from tick 0
```

### Concurrent Mode (dual - has issues)

**Setup:**
```cpp
BusArbiter arbiter(mem);

MemoryAdapter adapter0(mem, arbiter.cpu0_dport());
MemoryAdapter adapter1(mem, arbiter.cpu1_dport());

Cpu cpu0(adapter0, 0);
Cpu cpu1(adapter1, 1);

// Main loop
while (!cpu0.halted() || !cpu1.halted()) {
    cpu0.tick();
    cpu1.tick();
    arbiter.tick();
}
```

**How arbiter.tick() works:**

```cpp
void BusArbiter::tick() {
    DataRamPort& real = mem_.dport();
    real.tick();

    // 1. Route completed responses
    MemRsp rsp = real.poll();
    if (rsp.ready) {
        int cpu_id = pending_cpu_ids_.front();
        pending_cpu_ids_.pop();

        if (cpu_id == 0) {
            cpu0_dport_.completed_.push(rsp);
        } else {
            cpu1_dport_.completed_.push(rsp);
        }
    }

    // 2. Forward one request (round-robin)
    if (cpu0_dport_.has_request() && priority_ == 0) {
        // CPU0's turn
        MemReq req = cpu0_dport_.pending_.front();
        cpu0_dport_.pending_.pop();
        real.issue(req);
        pending_cpu_ids_.push(0);
        priority_ = 1;  // Switch to CPU1
    }
    else if (cpu1_dport_.has_request() && priority_ == 1) {
        // CPU1's turn
        MemReq req = cpu1_dport_.pending_.front();
        cpu1_dport_.pending_.pop();
        real.issue(req);
        pending_cpu_ids_.push(1);
        priority_ = 0;  // Switch to CPU0
    }
    // If current priority has no request, try other CPU
    else if (cpu1_dport_.has_request() && priority_ == 0) {
        // ... (don't switch priority)
    }
}
```

**Why it deadlocks:**
The MemoryAdapter uses `reinterpret_cast` to make VirtualDataPort look like DataRamPort. This is undefined behavior and likely corrupts the interface.

---

## Key Data Structures

### Pipeline Latches

```cpp
struct FetchLatch {
    bool valid;           // Is there an instruction here?
    u32 pc;              // PC of this instruction
    u32 instruction;     // 32-bit encoded instruction
};

struct DecodeLatch {
    bool valid;
    u32 pc;
    Decoded decoded;     // Decoded fields
    u32 rs1_val;         // Source operand values
    u32 rs2_val;
    f32 frs1_val;
    f32 frs2_val;
};

struct ExecuteLatch {
    bool valid;
    Decoded decoded;
    u32 result;              // Integer result
    f32 fresult;             // FP result
    int execute_countdown;   // Ticks remaining
    bool mem_pending;        // Waiting for memory?
    int mem_req_id;
};
```

### Decoded Instruction

```cpp
struct Decoded {
    enum class Kind {
        ADD, SUB, ADDI, LUI, AUIPC, SLLI,
        LW, SW, FLW, FSW,
        FADD, FSUB,
        BEQ, BNE, BLT,
        JAL, JALR,
        UNKNOWN
    } kind;

    u8 rd;      // Destination register (int)
    u8 rs1;     // Source register 1 (int)
    u8 rs2;     // Source register 2 (int)
    u8 frd;     // Destination register (FP)
    u8 frs1;    // Source register 1 (FP)
    u8 frs2;    // Source register 2 (FP)
    i32 imm;    // Immediate value

    bool writes_int;  // Does this write to int reg?
    bool writes_fp;   // Does this write to FP reg?
    bool is_branch;   // Is this a branch?
    bool is_jump;     // Is this a jump?
    bool is_load;     // Is this a load?
    bool is_store;    // Is this a store?
};
```

### Memory Request/Response

```cpp
struct MemReq {
    enum class Op { Read, Write } op;
    u32 addr;
    u32 wdata;  // For writes
    int id;     // Tracking ID
};

struct MemRsp {
    bool ready;   // Is response ready?
    u32 rdata;    // Read data
    int id;       // Which request this responds to
};
```

### Statistics

```cpp
struct Stats {
    u64 ticks;      // Simulation time (fine-grained)
    u64 cycles;     // CPU cycles (ticks / 10)
    u64 retired;    // Instructions completed
    u64 loads;      // Load operations
    u64 stores;     // Store operations
    u64 stalls;     // Stall cycles

    double cpi() const {
        return retired > 0 ?
            static_cast<double>(cycles) / retired : 0.0;
    }
};
```

---

## Common Code Patterns

### Pattern 1: Tick-Wait-Poll (Memory Operations)

```cpp
// In Execute stage

// Issue request
MemReq req;
req.op = MemReq::Op::Read;
req.addr = address;
req.id = stats_.retired;

if (!mem_.dport().can_issue()) {
    // Port busy - retry next tick
    decode_latch_.valid = true;  // Put instruction back
    stall_fetch_ = true;
    return;
}

mem_.dport().issue(req);
execute_latch_.mem_pending = true;
execute_latch_.execute_countdown = 20;

// Later ticks...
if (execute_latch_.mem_pending) {
    MemRsp rsp = mem_.dport().poll();
    if (!rsp.ready) {
        // Still waiting
        stall_decode_ = true;
        stall_fetch_ = true;
        return;
    }

    // Got response!
    execute_latch_.result = rsp.rdata;
    execute_latch_.mem_pending = false;
}
```

### Pattern 2: Scoreboard Check-Mark-Clear

```cpp
// In Decode: Check
if (int_busy_[rs1] || int_busy_[rs2]) {
    stall_fetch_ = true;
    return;  // Hazard!
}

// Mark destination busy
if (rd != 0) {
    int_busy_[rd] = true;
}

// In Writeback: Clear
int_busy_[rd] = false;
```

### Pattern 3: Pipeline Stall

```cpp
// In Execute (during multi-cycle op)
if (execute_countdown > 0) {
    execute_countdown--;
    if (execute_countdown > 0) {
        stall_decode_ = true;  // Prevent new instruction entering EX
        stall_fetch_ = true;   // Prevent new instruction entering ID
        return;
    }
}
```

### Pattern 4: Branch/Jump Flush

```cpp
// In Execute (branch taken)
if (branch_taken) {
    next_pc_ = branch_target;
    flush_fetch_ = true;   // Invalidate fetched instruction
    flush_decode_ = true;  // Invalidate decoded instruction
}

// In Fetch
if (flush_fetch_) {
    fetch_latch_.valid = false;
    return;
}

// In Decode
if (flush_decode_) {
    decode_latch_.valid = false;
    return;
}
```

### Pattern 5: %hi/%lo Address Calculation

```cpp
// For address 0xC00 (3072)
u32 addr = 0xC00;

// Extract high and low parts
u32 hi = addr & 0xFFFFF000;  // 0x0000
i32 lo = addr & 0xFFF;        // 0xC00

// Adjust for sign extension (bit 11 set?)
if (lo & 0x800) {
    hi += 0x1000;  // hi = 0x1000
    // lo will be sign-extended to -0x400
}

// Encode instructions
program.push_back(encode_u_type(0x37, rd, hi));
// lui rd, %hi  → rd = 0x00001000

program.push_back(encode_i_type(0x13, rd, 0x0, rd, lo));
// addi rd, rd, %lo  → rd = 0x1000 + (-0x400) = 0xC00 ✓
```

---

## Debugging Walkthrough

### Problem: CPU hangs, never halts

**Step 1: Add tick counter**
```cpp
if (ticks % 10000 == 0) {
    std::cout << "Tick " << ticks
              << " retired=" << stats_.retired
              << " PC=0x" << std::hex << pc_ << std::dec << "\n";
}
```

**Output:**
```
Tick 0 retired=0 PC=0x100
Tick 10000 retired=1184 PC=0x12c
Tick 20000 retired=2373 PC=0x15c
Tick 30000 retired=3560 PC=0x12c
...
Tick 1000000 retired=228569 PC=0x12c  ← WAY TOO MANY!
```

**Analysis:** Retired count growing without bound → infinite loop

**Step 2: Check loop counter**
```cpp
if (pc_ == 0x120) {  // Loop condition check
    u32 i = mem_.read_u32(0x2F0);  // -16(s0) with s0=0x300
    std::cout << "Loop counter i=" << i << "\n";
}
```

**Output:**
```
Loop counter i=0
Loop counter i=0
Loop counter i=0
...
```

**Analysis:** Loop counter not incrementing!

**Step 3: Check increment code**
```cpp
// Print encoded instructions around increment
for (int offset = 0x170; offset <= 0x17C; offset += 4) {
    u32 inst = mem_.ifetch(offset);
    std::cout << "0x" << std::hex << offset << ": 0x" << inst << "\n";
}
```

**Step 4: Find root cause**
- Compare with working vadd
- Discover different prologue (extra stores)
- Realize jump offsets are wrong
- Fix by matching vadd structure exactly

---

## Summary Checklist

When explaining the code, cover these key points:

✅ **Initialization:**
- Memory allocation (64KB)
- Program loading (encoding instructions)
- Array initialization (pseudo-random floats)

✅ **Pipeline Execution:**
- Reverse-order stage execution (WB→EX→ID→IF)
- Pipeline latches pass data between stages
- Tick-driven simulation (10 ticks = 1 cycle)

✅ **Memory System:**
- DataRamPort with request queue
- 20-tick (2-cycle) latency model
- Tick counter tracks simulation time

✅ **Hazard Handling:**
- Scoreboard tracks register busy status
- Stall signals prevent pipeline advance
- Flush signals clear pipeline stages

✅ **Instruction Encoding:**
- R/I/S/B/U/J type formats
- %hi/%lo splitting for large immediates
- Manual encoding of all instructions

✅ **Critical Details:**
- Execute countdown in TICKS not cycles
- Memory port reset between sequential runs
- Scoreboard mark in decode, clear in writeback
- Pipeline stalls on multi-cycle ops

This guide should help you explain any aspect of the code in detail! 🚀
