#include "cpu.hpp"
#include "timing.hpp"
#include <cstring>
#include <iostream>

Cpu::Cpu(Memory& mem, int cpu_id)
    : mem_(mem), cpu_id_(cpu_id) {
    regs_.fill(0);
    fregs_.fill(0.0f);
    int_busy_.fill(false);
    fp_busy_.fill(false);
}

void Cpu::reset(u32 pc0) {
    pc_ = pc0;
    next_pc_ = pc0;
    halted_ = false;

    regs_.fill(0);
    fregs_.fill(0.0f);
    int_busy_.fill(false);
    fp_busy_.fill(false);

    // Set up stack pointer (sp = x2) to top of stack region
    regs_[2] = 0x300;  // Stack grows down from 0x300

    fetch_latch_ = {};
    decode_latch_ = {};
    execute_latch_ = {};

    stats_ = {};
    stall_fetch_ = false;
    stall_decode_ = false;
    flush_fetch_ = false;
    flush_decode_ = false;
}

Stats Cpu::run(u64 max_ticks) {
    while (!halted_) {
        tick();

        if (max_ticks > 0 && stats_.ticks >= max_ticks) {
            break;
        }
    }
    return stats_;
}

void Cpu::tick() {
    // Advance memory port timing
    mem_.dport().tick();

    // Execute pipeline stages in reverse order (WB -> EX -> DE -> FE)
    // to avoid data race within the same tick
    stage_writeback();
    stage_execute();
    stage_decode();
    stage_fetch();

    // Update PC
    pc_ = next_pc_;

    // Clear flush flags
    flush_fetch_ = false;
    flush_decode_ = false;

    stats_.ticks++;
    if (stats_.ticks % timing::kTicksPerCycle == 0) {
        stats_.cycles++;
    }
}

void Cpu::stage_fetch() {
    if (halted_) return;

    if (flush_fetch_) {
        fetch_latch_.valid = false;
        return;
    }

    if (stall_fetch_) {
        stall_fetch_ = false;  // Clear stall for next tick
        return;
    }

    // Don't fetch if previous instruction hasn't been consumed
    if (fetch_latch_.valid) {
        return;
    }

    // Fetch instruction from memory
    u32 instr = mem_.ifetch(pc_);

    // Check for halt condition (e.g., RET instruction or infinite loop)
    // Using RET (jalr x0, 0(x1)) as halt: 0x00008067
    if (instr == 0x00008067) {
        halted_ = true;
        return;
    }

    fetch_latch_.valid = true;
    fetch_latch_.pc = pc_;
    fetch_latch_.instruction = instr;

    if (stats_.retired < 0) {
        std::cerr << "Fetch: PC=0x" << std::hex << pc_ << " instr=0x" << instr << std::dec << "\n";
    }

    // Update next PC (default: PC + 4)
    next_pc_ = pc_ + 4;
}

void Cpu::stage_decode() {
    if (flush_decode_) {
        if (stats_.retired < 0) {
            std::cerr << "Decode: FLUSHED\n";
        }
        decode_latch_.valid = false;
        return;
    }

    if (!fetch_latch_.valid || stall_decode_) {
        if (stats_.retired < 0 && stall_decode_) {
            std::cerr << "Decode: STALLED\n";
        }
        stall_decode_ = false;
        return;
    }

    // Decode instruction
    Decoded d = isa::decode(fetch_latch_.instruction);

    // Check for hazards
    if (check_hazard(d)) {
        stall_fetch_ = true;
        stall_decode_ = true;
        stats_.stalls++;
        return;
    }

    // Read register values
    u32 rs1_val = (d.rs1 == 0) ? 0 : regs_[d.rs1];
    u32 rs2_val = (d.rs2 == 0) ? 0 : regs_[d.rs2];
    f32 frs1_val = fregs_[d.rs1];
    f32 frs2_val = fregs_[d.rs2];

    if (stats_.retired < 0) {
        std::cerr << "Decode: PC=0x" << std::hex << fetch_latch_.pc
                  << " opcode=0x" << static_cast<int>(d.opcode)
                  << " rd=" << static_cast<int>(d.rd)
                  << " rs1=" << static_cast<int>(d.rs1) << "(val=0x" << rs1_val << ")"
                  << " rs2=" << static_cast<int>(d.rs2) << "(val=0x" << rs2_val << ")"
                  << " imm=" << d.imm << std::dec << "\n";
    }

    // Mark destination register as busy
    mark_busy(d);

    decode_latch_.valid = true;
    decode_latch_.pc = fetch_latch_.pc;
    decode_latch_.decoded = d;
    decode_latch_.rs1_val = rs1_val;
    decode_latch_.rs2_val = rs2_val;
    decode_latch_.frs1_val = frs1_val;
    decode_latch_.frs2_val = frs2_val;

    fetch_latch_.valid = false;
}

void Cpu::stage_execute() {
    // Handle ongoing execution
    if (execute_latch_.valid) {
        // Check if execute is still in progress
        if (execute_latch_.execute_countdown > 0) {
            execute_latch_.execute_countdown--;
            if (execute_latch_.execute_countdown > 0) {
                stall_decode_ = true;
                stall_fetch_ = true;
                return;  // Still executing
            }
        }

        // Check if memory operation is pending
        if (execute_latch_.mem_pending) {
            auto rsp = mem_.dport().poll();
            if (!rsp.ready) {
                stall_decode_ = true;
                stall_fetch_ = true;
                return;  // Still waiting for memory
            }

            // Memory response ready
            if (execute_latch_.decoded.kind == Decoded::Kind::LW ||
                execute_latch_.decoded.kind == Decoded::Kind::FLW) {
                execute_latch_.result = rsp.rdata;
            }
            execute_latch_.mem_pending = false;
        }

        // Execute stage complete, ready for writeback
        return;
    }

    // Start new execution
    if (!decode_latch_.valid) {
        return;
    }

    const auto& d = decode_latch_.decoded;
    u32 result = 0;
    f32 fresult = 0.0f;
    int countdown = 0;
    bool mem_pending = false;

    // Execute based on instruction type
    switch (d.kind) {
        // Integer ALU
        case Decoded::Kind::ADD:
            result = isa::exec_add(decode_latch_.rs1_val, decode_latch_.rs2_val);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SUB:
            result = isa::exec_sub(decode_latch_.rs1_val, decode_latch_.rs2_val);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::ADDI:
            result = isa::exec_add(decode_latch_.rs1_val, static_cast<u32>(d.imm));
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::AND:
            result = isa::exec_and(decode_latch_.rs1_val, decode_latch_.rs2_val);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::OR:
            result = isa::exec_or(decode_latch_.rs1_val, decode_latch_.rs2_val);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::XOR:
            result = isa::exec_xor(decode_latch_.rs1_val, decode_latch_.rs2_val);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::ANDI:
            result = isa::exec_and(decode_latch_.rs1_val, static_cast<u32>(d.imm));
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::ORI:
            result = isa::exec_or(decode_latch_.rs1_val, static_cast<u32>(d.imm));
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::XORI:
            result = isa::exec_xor(decode_latch_.rs1_val, static_cast<u32>(d.imm));
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SLLI:
            result = isa::exec_sll(decode_latch_.rs1_val, d.imm & 0x1F);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SRLI:
            result = isa::exec_srl(decode_latch_.rs1_val, d.imm & 0x1F);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SRAI:
            result = isa::exec_sra(static_cast<i32>(decode_latch_.rs1_val), d.imm & 0x1F);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SLT:
            result = isa::exec_slt(static_cast<i32>(decode_latch_.rs1_val), static_cast<i32>(decode_latch_.rs2_val));
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SLTI:
            result = isa::exec_slt(static_cast<i32>(decode_latch_.rs1_val), d.imm);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SLTU:
            result = isa::exec_sltu(decode_latch_.rs1_val, decode_latch_.rs2_val);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::SLTIU:
            result = isa::exec_sltu(decode_latch_.rs1_val, static_cast<u32>(d.imm));
            countdown = timing::kIExecuteCycles - 1;
            break;

        // LUI/AUIPC
        case Decoded::Kind::LUI:
            result = static_cast<u32>(d.imm);
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::AUIPC:
            result = decode_latch_.pc + static_cast<u32>(d.imm);
            countdown = timing::kIExecuteCycles - 1;
            break;

        // Floating-point ALU
        case Decoded::Kind::FADD_S:
            fresult = isa::exec_fadd_s(decode_latch_.frs1_val, decode_latch_.frs2_val);
            countdown = timing::kFExecuteCycles - 1;
            break;
        case Decoded::Kind::FSUB_S:
            fresult = isa::exec_fsub_s(decode_latch_.frs1_val, decode_latch_.frs2_val);
            countdown = timing::kFExecuteCycles - 1;
            break;
        case Decoded::Kind::FMUL_S:
            fresult = isa::exec_fmul_s(decode_latch_.frs1_val, decode_latch_.frs2_val);
            countdown = timing::kFExecuteCycles - 1;
            break;
        case Decoded::Kind::FDIV_S:
            fresult = isa::exec_fdiv_s(decode_latch_.frs1_val, decode_latch_.frs2_val);
            countdown = timing::kFExecuteCycles - 1;
            break;

        // Memory operations
        case Decoded::Kind::LW:
        case Decoded::Kind::FLW: {
            u32 addr = static_cast<u32>(static_cast<i32>(decode_latch_.rs1_val) + d.imm);
            if (addr >= 0xF0000000) {
                std::cerr << "LW/FLW address error at PC=" << std::hex << decode_latch_.pc
                          << " rs1=" << static_cast<int>(d.rs1) << " rs1_val=" << decode_latch_.rs1_val
                          << " imm=" << d.imm << " addr=" << addr << std::dec << "\n";
            }
            if (mem_.dport().can_issue()) {
                MemReq req;
                req.op = MemReq::Op::Read;
                req.addr = addr;
                req.wdata = 0;
                req.id = static_cast<int>(stats_.loads);
                mem_.dport().issue(req);
                mem_pending = true;
                stats_.loads++;
            } else {
                // Port busy, stall
                stall_decode_ = true;
                stall_fetch_ = true;
                return;
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        }

        case Decoded::Kind::SW: {
            u32 addr = static_cast<u32>(static_cast<i32>(decode_latch_.rs1_val) + d.imm);
            if (addr >= 0xF0000000) {
                std::cerr << "SW address calculation error: rs1=" << std::hex << decode_latch_.rs1_val
                          << " imm=" << d.imm << " addr=" << addr << std::dec << "\n";
            }
            if (mem_.dport().can_issue()) {
                MemReq req;
                req.op = MemReq::Op::Write;
                req.addr = addr;
                req.wdata = decode_latch_.rs2_val;
                req.id = static_cast<int>(stats_.stores);
                mem_.dport().issue(req);
                mem_pending = true;
                stats_.stores++;
            } else {
                // Port busy, stall
                stall_decode_ = true;
                stall_fetch_ = true;
                return;
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        }

        case Decoded::Kind::FSW: {
            u32 addr = static_cast<u32>(static_cast<i32>(decode_latch_.rs1_val) + d.imm);
            if (mem_.dport().can_issue()) {
                // Convert f32 to u32 for transmission
                u32 wdata;
                std::memcpy(&wdata, &decode_latch_.frs2_val, sizeof(u32));

                MemReq req;
                req.op = MemReq::Op::Write;
                req.addr = addr;
                req.wdata = wdata;
                req.id = static_cast<int>(stats_.stores);
                mem_.dport().issue(req);
                mem_pending = true;
                stats_.stores++;
            } else {
                // Port busy, stall
                stall_decode_ = true;
                stall_fetch_ = true;
                return;
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        }

        // Branches
        case Decoded::Kind::BEQ:
            if (isa::eval_beq(decode_latch_.rs1_val, decode_latch_.rs2_val)) {
                handle_branch(isa::calc_branch_target(decode_latch_.pc, d.imm));
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::BNE:
            if (isa::eval_bne(decode_latch_.rs1_val, decode_latch_.rs2_val)) {
                handle_branch(isa::calc_branch_target(decode_latch_.pc, d.imm));
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::BLT:
            if (isa::eval_blt(static_cast<i32>(decode_latch_.rs1_val), static_cast<i32>(decode_latch_.rs2_val))) {
                handle_branch(isa::calc_branch_target(decode_latch_.pc, d.imm));
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::BGE:
            if (isa::eval_bge(static_cast<i32>(decode_latch_.rs1_val), static_cast<i32>(decode_latch_.rs2_val))) {
                handle_branch(isa::calc_branch_target(decode_latch_.pc, d.imm));
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::BLTU:
            if (isa::eval_bltu(decode_latch_.rs1_val, decode_latch_.rs2_val)) {
                handle_branch(isa::calc_branch_target(decode_latch_.pc, d.imm));
            }
            countdown = timing::kIExecuteCycles - 1;
            break;
        case Decoded::Kind::BGEU:
            if (isa::eval_bgeu(decode_latch_.rs1_val, decode_latch_.rs2_val)) {
                handle_branch(isa::calc_branch_target(decode_latch_.pc, d.imm));
            }
            countdown = timing::kIExecuteCycles - 1;
            break;

        // Jumps
        case Decoded::Kind::JAL: {
            result = decode_latch_.pc + 4;  // Return address
            u32 target = isa::calc_jump_target(decode_latch_.pc, d.imm);
            if (stats_.retired < 30) {
                std::cerr << "JAL at PC=0x" << std::hex << decode_latch_.pc << " imm=" << std::dec << d.imm
                          << " target=0x" << std::hex << target << std::dec << "\n";
            }
            handle_branch(target);
            countdown = timing::kIExecuteCycles - 1;
            break;
        }
        case Decoded::Kind::JALR:
            result = decode_latch_.pc + 4;  // Return address
            handle_branch(isa::calc_jalr_target(decode_latch_.rs1_val, d.imm));
            countdown = timing::kIExecuteCycles - 1;
            break;

        case Decoded::Kind::NOP:
            countdown = timing::kIExecuteCycles - 1;
            break;

        default:
            std::cerr << "Unknown instruction kind at PC=" << std::hex << decode_latch_.pc << std::dec << "\n";
            halted_ = true;
            return;
    }

    execute_latch_.valid = true;
    execute_latch_.pc = decode_latch_.pc;
    execute_latch_.decoded = d;
    execute_latch_.result = result;
    execute_latch_.fresult = fresult;
    execute_latch_.execute_countdown = countdown;
    execute_latch_.mem_pending = mem_pending;

    decode_latch_.valid = false;
}

void Cpu::stage_writeback() {
    if (!execute_latch_.valid) {
        return;
    }

    // Can't writeback if still executing or waiting for memory
    if (execute_latch_.execute_countdown > 0 || execute_latch_.mem_pending) {
        return;
    }

    const auto& d = execute_latch_.decoded;

    // Write result to destination register
    if (d.kind == Decoded::Kind::LW) {
        if (d.rd != 0) {
            regs_[d.rd] = execute_latch_.result;
            if (stats_.retired < 0) {
                std::cerr << "  Write: x" << static_cast<int>(d.rd) << " = 0x" << std::hex << execute_latch_.result << std::dec << "\n";
            }
        }
    } else if (d.kind == Decoded::Kind::FLW) {
        std::memcpy(&fregs_[d.rd], &execute_latch_.result, sizeof(f32));
    } else if (d.is_fp() && d.rd != 0) {
        fregs_[d.rd] = execute_latch_.fresult;
    } else if (d.rd != 0 && !d.is_branch() && !d.is_memory()) {
        regs_[d.rd] = execute_latch_.result;
        if (stats_.retired < 0) {
            std::cerr << "  Write: x" << static_cast<int>(d.rd) << " = 0x" << std::hex << execute_latch_.result << std::dec << "\n";
        }
    }

    // Clear busy flag
    clear_busy(d);

    // Retire instruction
    stats_.retired++;

    if (stats_.retired < 50) {
        std::cerr << "Retired #" << stats_.retired << " at PC=" << std::hex << execute_latch_.pc;
        if (execute_latch_.decoded.kind == Decoded::Kind::JAL || execute_latch_.decoded.kind == Decoded::Kind::BEQ ||
            execute_latch_.decoded.kind == Decoded::Kind::BNE || execute_latch_.decoded.kind == Decoded::Kind::BLT) {
            std::cerr << " (branch/jump to PC=" << next_pc_ << ")";
        }
        std::cerr << std::dec << "\n";
    }

    execute_latch_.valid = false;
}

bool Cpu::check_hazard(const Decoded& d) {
    // Check if source registers are busy
    if (d.rs1 != 0) {
        if (d.is_fp()) {
            if (fp_busy_[d.rs1]) return true;
        } else {
            if (int_busy_[d.rs1]) return true;
        }
    }

    if (d.rs2 != 0) {
        // SW/FSW use rs2 for data
        if (d.kind == Decoded::Kind::SW && int_busy_[d.rs2]) return true;
        if (d.kind == Decoded::Kind::FSW && fp_busy_[d.rs2]) return true;

        // R-type and branches use rs2
        if ((d.kind == Decoded::Kind::ADD || d.kind == Decoded::Kind::SUB ||
             d.kind == Decoded::Kind::AND || d.kind == Decoded::Kind::OR ||
             d.kind == Decoded::Kind::XOR || d.kind == Decoded::Kind::SLT ||
             d.kind == Decoded::Kind::SLTU || d.is_branch())) {
            if (int_busy_[d.rs2]) return true;
        }

        // FP R-type uses frs2
        if (d.kind == Decoded::Kind::FADD_S || d.kind == Decoded::Kind::FSUB_S ||
            d.kind == Decoded::Kind::FMUL_S || d.kind == Decoded::Kind::FDIV_S) {
            if (fp_busy_[d.rs2]) return true;
        }
    }

    return false;
}

void Cpu::mark_busy(const Decoded& d) {
    if (d.rd == 0) return;

    // Stores and branches don't write to rd
    if (d.kind == Decoded::Kind::SW || d.kind == Decoded::Kind::FSW || d.is_branch()) {
        return;
    }

    if (d.is_fp()) {
        fp_busy_[d.rd] = true;
    } else {
        int_busy_[d.rd] = true;
    }
}

void Cpu::clear_busy(const Decoded& d) {
    if (d.rd != 0) {
        if (d.is_fp()) {
            fp_busy_[d.rd] = false;
        } else {
            int_busy_[d.rd] = false;
        }
    }
}

void Cpu::handle_branch(u32 target) {
    next_pc_ = target;
    flush_fetch_ = true;
    flush_decode_ = true;
}
