# Architecture Q&A Reference Guide
## Questions and Answers for TA/Professor Discussion

---

## General Architecture Questions

### Q1: What is the overall architecture of your simulator?
**A:** The simulator implements a dual-CPU RISC-V RV32I/F system with a 4-stage pipeline per CPU. Each CPU has separate instruction ports but shares a single data memory port through a round-robin bus arbiter. The system is tick-driven with 10 ticks per CPU cycle, modeling variable-latency execution (1 cycle for integer ops, 5 cycles for floating-point) and 2-cycle memory latency.

### Q2: Why did you choose a 4-stage pipeline instead of the classic 5-stage?
**A:** The assignment specification called for a 4-stage pipeline: Fetch, Decode, Execute, and Writeback. We combined Memory Access into the Execute stage since memory operations are handled through the same port as data operations, and the 2-cycle memory latency is modeled during execute countdown.

### Q3: What is the difference between ticks and cycles in your simulator?
**A:**
- **1 Cycle = 10 Ticks** (configurable in `timing.hpp`)
- Ticks are the fundamental unit of simulation time
- Cycles represent CPU clock cycles
- All latencies are specified in cycles but implemented as tick countdowns
- Example: A 5-cycle FP operation = 50 ticks of countdown

---

## Pipeline Questions

### Q4: Walk me through how an instruction flows through your pipeline.
**A:**
1. **Fetch (IF)**: Read instruction from memory at PC, latch into IF/ID register, advance PC by 4
2. **Decode (ID)**: Extract opcode/registers, read source operands, check scoreboard for hazards
3. **Execute (EX)**: Perform ALU/FPU operation or issue memory request, countdown ticks based on operation latency
4. **Writeback (WB)**: Write result to destination register, clear scoreboard busy bit

The instruction remains in each stage for at least one tick, but Execute can take multiple ticks (10 for int, 50 for FP, 20+ for memory).

### Q5: How do you handle pipeline hazards?
**A:**
**Structural Hazards:**
- Data port busy → stall decode and fetch stages until port available
- Check `can_issue()` before issuing memory requests

**Data Hazards (RAW):**
- Scoreboard tracking: mark destination register busy when instruction enters Execute
- Stall Decode if source register is busy
- Clear busy bit in Writeback
- No forwarding implemented (conservative approach)

**Control Hazards:**
- Branch/jump taken → flush IF and ID stages
- Redirect PC to target address
- Penalty: 2 cycles (flushed instructions)

### Q6: Why don't you implement forwarding?
**A:** The assignment didn't require forwarding, and scoreboard-based stalling is simpler and still correct. Forwarding would reduce CPI by eliminating some RAW stalls, but adds complexity in routing results from EX→ID. Our CPI of 1.688 is reasonable without forwarding.

### Q7: How do you model multi-cycle operations?
**A:**
- Each operation has a tick countdown (stored in `execute_countdown`)
- Integer ops: 10 ticks (1 cycle)
- Floating-point ops: 50 ticks (5 cycles)
- Memory ops: 20 ticks (2 cycles) minimum
- Countdown decrements every tick
- Pipeline stalls (can't advance to WB) while countdown > 0

---

## Memory System Questions

### Q8: Explain your memory port architecture.
**A:**
We have two types of ports:
1. **I-Port (Instruction Port)**:
   - Read-only, instant access
   - Separate port per CPU (no contention)

2. **D-Port (Data Port)**:
   - Read/Write with 2-cycle (20 tick) latency
   - Queue-based: up to 8 outstanding requests
   - Shared between CPUs (requires arbitration in Part 2)

### Q9: How does memory latency work?
**A:**
When a CPU issues a memory request:
1. Check if port `can_issue()` (queue not full)
2. Call `issue(request)` → queued with `completion_tick = current_tick + 20`
3. Port ticks forward, incrementing `current_tick`
4. When `current_tick >= completion_tick`, request completes
5. CPU polls with `poll()` to retrieve response
6. Execute stage waits for `response.ready == true`

**This models the round-trip time of accessing RAM.**

### Q10: What happens if the memory port is busy?
**A:**
The CPU **retries** the memory request next tick:
```cpp
if (!mem_.dport().can_issue()) {
    // Keep instruction in EX stage, retry next tick
    stall_decode_ = true;
    stall_fetch_ = true;
    return;
}
```
This creates a structural hazard stall until the port has queue space.

### Q11: Why is there a memory port reset between CPU runs in sequential mode?
**A:**
The DataRamPort maintains a `current_tick_` counter that accumulates. After CPU0 finishes (say, at tick 51,870), if CPU1 issues a request, it gets `completion_tick = 51,870 + 20 = 51,890`. But CPU1 just started ticking from 0, so it would never reach tick 51,890! Solution: `mem.dport().reset()` clears the tick counter before CPU1 starts.

---

## Bus Arbitration Questions

### Q12: How does your bus arbiter work?
**A:**
**Round-Robin Arbitration:**
- Maintains a `priority_` bit (0 or 1) indicating which CPU has priority
- Each tick:
  1. Tick the real memory port (advance latency)
  2. Poll for completed requests, route to correct CPU
  3. Forward **one request** from virtual ports to real port
     - Try priority CPU first
     - If successful, switch priority to other CPU
     - If priority CPU has no request, try other CPU (keep priority)

This ensures fairness: CPUs alternate getting access to memory.

### Q13: What is a VirtualDataPort?
**A:**
Each CPU gets a VirtualDataPort that looks like a DataRamPort but doesn't actually access memory. It has:
- `pending_` queue: requests waiting to be forwarded
- `completed_` queue: responses from real memory
- `can_issue()`, `issue()`, `poll()` methods matching DataRamPort interface

The BusArbiter forwards requests from virtual→real and responses from real→virtual.

### Q14: Why does the concurrent dual-CPU version deadlock?
**A:**
The issue is the MemoryAdapter using `reinterpret_cast<DataRamPort&>(VirtualDataPortWrapper&)`. While VirtualDataPortWrapper has an identical interface to DataRamPort, the reinterpret_cast is undefined behavior and likely causes vtable or memory layout issues. The CPU thinks it's calling DataRamPort methods but actually corrupts memory or calls the wrong functions.

**Workaround:** Sequential mode (dual_simple) avoids this by running CPUs one at a time.

### Q15: How would you fix the concurrent mode?
**A:**
Three options:
1. **Template-based CPU**: `template<typename PortType> class Cpu` - compile-time polymorphism
2. **Virtual inheritance**: Make DataRamPort and VirtualDataPort inherit from common `IDataPort` interface
3. **Function pointers**: Store function pointers instead of casting types

Option 2 is cleanest architecturally.

---

## ISA and Encoding Questions

### Q16: Which RISC-V instructions do you support?
**A:**
**RV32I**: LUI, AUIPC, ADDI, SLLI, ADD, SUB, LW, SW, BEQ, BNE, BLT, JAL, JALR
**RV32F**: FLW, FSW, FADD.S, FSUB.S

These cover all instructions needed for vadd and vsub programs. We don't implement the full RV32I/F spec, only what's necessary.

### Q17: How do you encode instructions?
**A:**
We have encoding functions for each instruction format:
```cpp
encode_r_type(opcode, rd, funct3, rs1, rs2, funct7);  // ADD, FADD.S
encode_i_type(opcode, rd, funct3, rs1, imm);          // ADDI, LW
encode_s_type(opcode, funct3, rs1, rs2, imm);         // SW, FSW
encode_b_type(opcode, funct3, rs1, rs2, imm);         // BLT, BEQ
encode_u_type(opcode, rd, imm);                       // LUI
encode_j_type(opcode, rd, imm);                       // JAL
```

These pack the fields into a 32-bit instruction following RISC-V spec.

### Q18: Explain the %hi/%lo mechanism.
**A:**
RISC-V immediate fields are only 12 bits, but we need 32-bit addresses. Solution: split into high 20 bits and low 12 bits:

**Example: Load from 0xC00**
```asm
lui  a0, %hi(0xC00)     # Load upper: a0 = %hi << 12
addi a0, a0, %lo(0xC00) # Add lower: a0 += %lo
```

**Calculation:**
```cpp
uint32_t addr = 0xC00;
uint32_t hi = addr & 0xFFFFF000;        // 0x0000
int32_t  lo = (int32_t)(addr & 0xFFF);  // 0xC00

// Sign-extension adjustment
if (lo & 0x800) {  // Bit 11 set?
    hi += 0x1000;  // Adjust: hi = 0x1000, lo = -0x400
}

// Result: lui loads 0x1000, addi adds -0x400 → 0xC00 ✓
```

### Q19: How do you handle branch/jump offsets?
**A:**
Offsets are **relative to PC** and **in bytes**:
```cpp
// JAL back to loop from position 31 to position 8
u32 jump_from = 31;  // Current instruction index
u32 jump_target = 8; // Target instruction index
i32 byte_offset = (static_cast<i32>(jump_target) -
                   static_cast<i32>(jump_from)) * 4;
// Result: (8-31)*4 = -92 bytes

program.push_back(encode_j_type(0x6F, 0, byte_offset));
```

**Key:** Cast to signed before subtraction for negative offsets!

---

## Program-Specific Questions

### Q20: Walk me through the vadd program.
**A:**
```
1. Prologue: Save ra, s0 to stack, setup frame pointer
2. Initialize: i = 0 (stored at -16(s0))
3. Loop:
   a. Check: if i >= 256, jump to done
   b. Calculate: addr_A = 0x400 + i*4
   c. Load: A[i] into ft0
   d. Calculate: addr_B = 0x800 + i*4
   e. Load: B[i] into ft1
   f. Compute: ft0 = ft0 + ft1 (FADD.S, 5 cycles)
   g. Calculate: addr_C = 0xC00 + i*4
   h. Store: ft0 to C[i]
   i. Increment: i++
   j. Jump: back to loop
4. Epilogue: Restore ra, s0, return
```

**Total per iteration**: ~24 instructions
**Total iterations**: 256
**Total instructions**: ~6,157

### Q21: What's different between vadd and vsub?
**A:**
Only **3 differences**:
1. **Start address**: vadd at 0x000, vsub at 0x100
2. **FP operation**: vadd uses `fadd.s`, vsub uses `fsub.s`
3. **Store address**: vadd stores to C (0xC00), vsub stores to D (0x1000)

Everything else (prologue, loop structure, epilogue) is identical.

### Q22: Why do you initialize both -12(s0) and -16(s0)?
**A:**
From the assembly reference (CPU0.s, CPU1.s):
- `-12(s0)`: Return value slot (initialized to 0)
- `-16(s0)`: Loop counter `i` (initialized to 0, incremented each iteration)

Initially we only initialized -16(s0), but the epilogue loads from -12(s0), so we need to initialize it to avoid undefined behavior.

---

## Performance Questions

### Q23: What CPI do you achieve and why?
**A:**
**CPI = 1.688** for both vadd and vsub

**Breakdown:**
- 6,157 instructions retired
- 10,390 cycles elapsed

**Why 1.688?**
- ~5,900 integer ops × 1 cycle = 5,900 cycles
- 256 FP ops × 5 cycles = 1,280 cycles
- 512 memory ops × 2 cycles = 1,024 cycles
- Pipeline stalls from hazards ≈ 2,186 cycles
- **Total ≈ 10,390 cycles**

CPI > 1.0 is expected because:
1. FP operations take 5 cycles each
2. Memory operations stall for latency
3. RAW hazards cause decode stalls (no forwarding)

### Q24: How does bus contention affect CPI in dual-CPU mode?
**A:**
In **sequential mode**: Each CPU runs independently → CPI = 1.688 each

In **concurrent mode** (if it worked):
- Both CPUs compete for data port
- More queue full stalls → higher CPI
- Expected CPI ≈ 3-5 due to contention

The round-robin arbiter ensures fairness but doesn't eliminate contention.

### Q25: Could you achieve better CPI? How?
**A:**
Yes, several optimizations:

1. **Forwarding**: EX→ID bypass reduces RAW stalls
   - Estimated improvement: CPI → 1.2-1.3

2. **Out-of-order execution**: Issue independent instructions concurrently
   - Estimated improvement: CPI → 0.8-1.0

3. **Superscalar**: Multiple issue (2-way)
   - Estimated improvement: CPI → 0.5-0.7

4. **Better memory system**: Cache hierarchy
   - Reduces memory latency impact

5. **Branch prediction**: Reduce control hazard penalty
   - Saves 2 cycles on mispredicts

---

## Validation Questions

### Q26: How do you validate correctness?
**A:**
After simulation, we:
1. Read arrays A, B, C, D from memory
2. Compute reference results on host CPU:
   - `C_ref[i] = A[i] + B[i]`
   - `D_ref[i] = A[i] - B[i]`
3. Compare with tolerance:
   - Floating-point: 1 ULP tolerance
   - Check all 256 elements

**Validation passes** if all elements match within tolerance.

### Q27: What is ULP tolerance and why use it?
**A:**
**ULP** = Unit in Last Place

Floating-point arithmetic isn't exact due to rounding. Two approaches to compute `A + B` might differ in the last bit.

**1 ULP tolerance** means the bit patterns can differ by at most 1:
```cpp
uint32_t a_bits, b_bits;
memcpy(&a_bits, &a, 4);
memcpy(&b_bits, &b, 4);
uint32_t diff = abs(a_bits - b_bits);
return diff <= 1;  // Allow 1 bit difference
```

This is more robust than absolute tolerance (e.g., `abs(a-b) < 1e-6`) for large or small numbers.

### Q28: How are arrays initialized?
**A:**
Pseudo-random floats using C++ `<random>`:
```cpp
std::mt19937 rng(1);  // Fixed seed for reproducibility
std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

for (int i = 0; i < 256; i++) {
    float a_val = dist(rng);
    float b_val = dist(rng);
    mem.write_f32(0x400 + i*4, a_val);  // Array A
    mem.write_f32(0x800 + i*4, b_val);  // Array B
}
```

**Fixed seed** ensures reproducible results across runs.

---

## Implementation Questions

### Q29: What's the most challenging bug you fixed?
**A:**
**Memory port state not resetting** between sequential CPU runs.

**Symptom:** CPU0 worked perfectly (6,157 instructions), but CPU1 hung forever after 1 instruction.

**Root cause:** The DataRamPort has `current_tick_` that accumulated to ~52,000 after CPU0 finished. When CPU1 issued a memory request at tick 0, it got `completion_tick = 52,020`. But CPU1's tick counter starts at 0, so it never reaches 52,020!

**Fix:** Added `DataRamPort::reset()` to clear state:
```cpp
void DataRamPort::reset() {
    current_tick_ = 0;
    while (!pending_.empty()) pending_.pop();
}
```

### Q30: What about the execute countdown bug?
**A:**
**Initial bug:** Countdown was in **cycles** but decremented every **tick**.

```cpp
// Wrong
countdown = kFExecuteCycles - 1;  // 5-1 = 4
// Decremented every tick → 4 ticks, not 50!

// Correct
countdown = cycles_to_ticks(kFExecuteCycles);  // 5*10 = 50
// Decremented every tick → 50 ticks ✓
```

This caused FP operations to complete in 4 ticks instead of 50, making CPI impossibly low (0.4 instead of 1.6).

### Q31: How did you debug the vsub infinite loop?
**A:**
Created a standalone test that ran vsub by itself. Discovered it completed fine alone but hung after CPU0 in dual mode. This isolated the problem to memory state, not the vsub encoding. Added debug output showing:
- CPU0: 6,157 retired ✓
- CPU1: 228,569 retired after 1M ticks ✗

The massive instruction count meant the loop never terminated → suggested the loop counter wasn't incrementing or branch condition was wrong. But standalone vsub worked, so it had to be a state issue → led to discovering the tick counter problem.

---

## Design Decisions Questions

### Q32: Why use tick-driven simulation instead of cycle-driven?
**A:**
**Tick-driven** provides finer granularity for modeling:
- Variable latencies (1 cycle vs 5 cycles) decompose into ticks
- Memory latency (20 ticks) can be counted down precisely
- Bus arbitration decisions happen every tick
- More accurate timing model

**Cycle-driven** would require fractional cycles or complex state machines.

### Q33: Why separate integer and FP register files?
**A:**
RISC-V RV32F spec defines separate register files:
- **x0-x31**: Integer registers (32-bit)
- **f0-f31**: Floating-point registers (32-bit single-precision)

This matches real hardware and avoids type confusion. Instructions specify which file they access:
- `add x10, x11, x12` → integer file
- `fadd.s f10, f11, f12` → FP file

### Q34: Why use a scoreboard instead of forwarding?
**A:**
**Simplicity vs Performance tradeoff:**

**Scoreboard:**
- ✅ Simple to implement (2 arrays of bools)
- ✅ Guarantees correctness (conservative)
- ❌ Higher CPI (more stalls)

**Forwarding:**
- ✅ Lower CPI (fewer stalls)
- ❌ Complex bypass network
- ❌ More bug-prone

For a course project, correctness > performance, so scoreboard was chosen.

### Q35: Why hardcode programs instead of loading ELF binaries?
**A:**
**Scope limitation:**
- ELF parsing adds complexity (symbol tables, relocations, segments)
- Would need a full assembler/linker toolchain
- Assignment only requires two specific programs (vadd, vsub)

**Current approach:**
- Programs encoded directly in `loader.cpp`
- Guaranteed correct encoding
- Easier to debug (can print encoded instructions)
- Sufficient for assignment requirements

---

## Comparison Questions

### Q36: How does your architecture differ from a real RISC-V processor?
**A:**

| Feature | Our Simulator | Real RISC-V |
|---------|---------------|-------------|
| **Pipeline** | 4-stage | 5+ stages (often 7-13) |
| **Issue** | Single | Superscalar (2-4 wide) |
| **Order** | In-order | Often out-of-order |
| **Forwarding** | None | Multiple bypass paths |
| **Cache** | None (direct RAM) | L1/L2/L3 hierarchy |
| **Branch Pred** | None | Advanced predictors |
| **FP Latency** | 5 cycles | 3-5 cycles (pipelined) |

Our simulator is **educational**, not performance-accurate.

### Q37: What's the difference between your bus arbiter and a real one?
**A:**
**Our arbiter:**
- Round-robin, 2 requesters
- One request per tick
- Simple priority bit

**Real bus arbiters:**
- Multiple priority levels
- Weighted fair queuing
- Pipelining (accept new request while servicing old)
- Split transactions (address/data phases)
- Burst transfers
- QoS (Quality of Service) guarantees

---

## Testing Questions

### Q38: What tests did you write?
**A:**
1. **ISA Tests**: Encoding/decoding correctness for each instruction type
2. **Memory Tests**: Latency accuracy (exactly 20 ticks), port independence
3. **CPU Tests**: Single instruction retirement, hazard stalling, branch behavior
4. **Integration Tests**: Full vadd/vsub execution with validation

**Framework:** Google Test (if available) or custom assertions

### Q39: How do you test pipeline hazards?
**A:**
Create synthetic instruction sequences:

**RAW Hazard:**
```cpp
// Test: consecutive dependent adds
memory.write(0x000, encode_addi(x10, x0, 5));   // x10 = 5
memory.write(0x004, encode_addi(x11, x10, 3));  // x11 = x10 + 3 (RAW!)

// Run for N ticks
cpu.run(100);

// Assert: x11 = 8, and extra stall cycles detected
ASSERT_EQ(cpu.get_reg(11), 8);
ASSERT_GT(cpu.cycles(), 2);  // More than 2 cycles due to stall
```

### Q40: How do you verify memory latency is exactly 2 cycles?
**A:**
```cpp
// Issue load at tick 0
mem.dport().issue({MemReq::Op::Read, 0x400, 0, 0});

// Tick forward, poll each tick
int tick = 0;
MemRsp rsp;
do {
    mem.dport().tick();
    rsp = mem.dport().poll();
    tick++;
} while (!rsp.ready);

// Assert: ready at exactly tick 20 (2 cycles)
ASSERT_EQ(tick, 20);
```

---

## Advanced Questions

### Q41: How would you extend this to 4 CPUs?
**A:**
1. **Bus Arbiter**: Generalize to N CPUs with round-robin over 4
   ```cpp
   std::vector<VirtualDataPort> cpu_ports(4);
   int priority = 0;
   // Round-robin: try priority, priority+1, ..., priority+3 (mod 4)
   ```

2. **Memory Adapter**: Create array of adapters
   ```cpp
   std::array<MemoryAdapter, 4> adapters;
   ```

3. **More Contention**: With 4 CPUs, expect 4-8× higher CPI due to bus stalls

### Q42: What if you wanted to model cache?
**A:**
Add a cache layer between CPU and RAM:

```
CPU → L1 Cache (1-cycle hit, 10-cycle miss) → DataRamPort → RAM
```

**Implementation:**
- Replace direct `mem_.dport()` calls with `cache.access()`
- Cache hit → return immediately
- Cache miss → issue RAM request, wait, fill cache line
- Track hit/miss statistics

**Impact:** Dramatically reduces effective memory latency (90%+ hit rate → ~1.1 cycles avg)

### Q43: How would you add branch prediction?
**A:**
**Simple predictor** (1-bit):
```cpp
std::unordered_map<u32, bool> branch_history;  // PC → last outcome

// In decode:
if (is_branch) {
    bool prediction = branch_history[pc];  // Default: false

    if (prediction) {
        // Speculatively fetch from target
        next_pc = pc + branch_offset;
    } else {
        // Fetch from PC+4
        next_pc = pc + 4;
    }
}

// In execute (when resolved):
if (actual_outcome != prediction) {
    // Mispredict: flush pipeline, correct PC
    flush_pipeline();
    branch_history[pc] = actual_outcome;  // Update
}
```

### Q44: Could you simulate interrupt handling?
**A:**
Yes, add:
1. **CSR registers**: mstatus, mtvec, mepc, mcause
2. **Interrupt signal**: External pin or timer
3. **Exception handling**:
   - Save PC to mepc
   - Save cause to mcause
   - Jump to mtvec (trap handler)
   - Execute handler code
   - mret instruction to restore

**Complexity:** Requires privilege levels (M-mode) and CSR instructions.

### Q45: What about multi-threading (SMT)?
**A:**
**Simultaneous Multi-Threading** = multiple hardware threads per core

**Changes needed:**
- Duplicate architectural state (PC, registers) per thread
- Share execution units (ALU, FPU)
- Thread scheduler in fetch stage
- Tag pipeline stages with thread ID

**Benefit:** Hide latency (while thread 0 waits for memory, thread 1 executes)

---

## Debugging and Tools Questions

### Q46: What tools did you use for debugging?
**A:**
1. **Print debugging**: Trace PC, instruction retirement
2. **Instruction dumps**: Print encoded program to verify correctness
3. **Standalone tests**: Isolate components (CPU0 alone, CPU1 alone)
4. **Assertions**: Validate state at key points
5. **Tick-by-tick tracing**: Print every pipeline stage transition

### Q47: How do you print the encoded program?
**A:**
```cpp
Memory mem(64*1024);
loader::load_program_vsub(mem, map);

for (int i = 0; i < 40; i++) {
    u32 addr = 0x100 + i*4;
    u32 inst = mem.ifetch(addr);
    std::cout << "0x" << std::hex << addr << ": 0x"
              << std::setw(8) << std::setfill('0') << inst << "\n";
}
```

Compare against reference assembly to verify encoding.

### Q48: What's your process for finding a bug?
**A:**
1. **Reproduce**: Create minimal test case that triggers bug
2. **Isolate**: Binary search (disable half the code, see if bug persists)
3. **Trace**: Add debug output at suspected locations
4. **Hypothesis**: Form theory about root cause
5. **Test**: Add assertion to validate hypothesis
6. **Fix**: Implement fix, verify with regression tests

**Example:** vsub hanging → isolated to "works alone, fails after CPU0" → hypothesis: "shared state" → found tick counter → fixed with reset.

---

## Conceptual Questions

### Q49: What is the difference between latency and throughput?
**A:**
**Latency**: Time for one operation to complete
- Memory load: 20 ticks (2 cycles)
- FP add: 50 ticks (5 cycles)

**Throughput**: Operations completed per unit time
- With pipelining: can complete 1 instruction per cycle (steady state)
- Even though FP add takes 5 cycles, next FP add can start in cycle 2

**Pipelining increases throughput without reducing latency.**

### Q50: Explain instruction-level parallelism (ILP).
**A:**
**ILP** = Ability to execute multiple instructions concurrently

**Our simulator:**
- Limited ILP (only 4 instructions in pipeline at once)
- No superscalar (only 1 issue per cycle)
- In-order execution

**High-ILP processor:**
- Superscalar: issue 4+ instructions per cycle
- Out-of-order: execute independent instructions in any order
- Speculative: guess branches and execute ahead

**Benefit:** CPI < 1 (e.g., CPI = 0.5 means 2 instructions per cycle average)

---

## Project Management Questions

### Q51: What was your development process?
**A:**
1. **Part 1**: Single CPU
   - Implement ISA encoding
   - Build memory system with latency
   - Implement 4-stage pipeline
   - Test with vadd
   - Validate results

2. **Part 2**: Dual CPU
   - Design bus arbiter (round-robin)
   - Create VirtualDataPort abstraction
   - Implement memory adapter
   - Encode vsub program
   - Debug sequential mode first
   - Attempt concurrent mode (encountered deadlock)

### Q52: What would you do differently?
**A:**
1. **Use polymorphism from the start**: Define `IMemoryPort` interface instead of concrete `DataRamPort` to avoid reinterpret_cast issues

2. **More unit tests earlier**: Catch encoding bugs before integration

3. **Visualizer tool**: Build a waveform viewer to see pipeline state graphically

4. **Modular design**: Separate timing model from functional model for easier testing

### Q53: What did you learn from this project?
**A:**
1. **Pipeline hazards are subtle**: Easy to miss corner cases (RAW on FP registers)
2. **Timing models are complex**: Tick-driven simulation requires careful accounting
3. **Type safety matters**: reinterpret_cast caused major debugging pain
4. **Validation is critical**: Without ULP-tolerant FP comparison, false failures everywhere
5. **State management**: Resetting state between runs is essential in sequential simulations

---

## Bonus: Tricky Questions

### Q54: If I change memory latency to 1 cycle, does CPI improve?
**A:**
**Yes, but less than you'd expect.**

Current: 512 memory ops × 2 cycles = 1,024 cycles
New: 512 memory ops × 1 cycle = 512 cycles
**Savings: 512 cycles**

Expected new CPI: (10390 - 512) / 6157 = **1.605** (vs current 1.688)

**Why not more improvement?**
- Pipeline stalls from hazards still occur
- FP latency (5 cycles) dominates for 256 operations

### Q55: What if FP operations were 1 cycle instead of 5?
**A:**
**Huge improvement.**

Current: 256 FP ops × 5 cycles = 1,280 cycles
New: 256 FP ops × 1 cycle = 256 cycles
**Savings: 1,024 cycles**

Expected new CPI: (10390 - 1024) / 6157 = **1.521**

**In reality:** Even better because fewer hazard stalls (FP results available sooner)
**Realistic CPI: ~1.2-1.3**

### Q56: Can CPI ever be less than 1.0 in your simulator?
**A:**
**No, not with current architecture.**

CPI < 1 requires **more than 1 instruction completing per cycle** on average. This needs:
- Superscalar (multiple issue)
- Out-of-order execution
- VLIW (Very Long Instruction Word)

Our single-issue, in-order pipeline can at best achieve CPI = 1.0 in ideal conditions (no hazards, no multi-cycle ops).

### Q57: Your friend's code shows CPI=10. Is that realistic?
**A:**
**No, their calculation is wrong.**

They compute: `CPI = ticks / cycles = 100,000 / 10,000 = 10`

But this is **ticks-per-cycle**, which is always 10 by definition!

**Real CPI** should be: `CPI = cycles / instructions_retired`

Their code doesn't track `instructions_retired`, so they can't compute actual CPI. Our CPI of 1.688 is the correct metric.

---

## Summary

**Key Points to Remember:**

1. **Architecture**: 4-stage pipeline, tick-driven, 10 ticks per cycle
2. **Latencies**: 1 cycle (int), 5 cycles (FP), 2 cycles (memory)
3. **Hazards**: Scoreboard for data, flush for control, stall for structural
4. **CPI**: 1.688 due to multi-cycle ops and hazards
5. **Part 2**: Round-robin bus arbiter, sequential mode works, concurrent has deadlock
6. **Validation**: 1 ULP tolerance for FP, all 256 elements checked

**Be ready to explain:**
- Pipeline flow for a specific instruction
- How scoreboard prevents RAW hazards
- Why memory port needs reset between sequential runs
- Encoding of a specific instruction (e.g., JAL)
- Bus arbitration decision process

Good luck with your presentation! 🎓
