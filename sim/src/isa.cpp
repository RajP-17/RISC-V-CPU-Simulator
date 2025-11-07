#include "isa.hpp"
#include <cstring>

namespace isa {

i32 sign_extend(u32 value, int bits) {
    u32 sign_bit = 1u << (bits - 1);
    if (value & sign_bit) {
        // Negative: set upper bits to 1
        u32 mask = ~((1u << bits) - 1);
        return static_cast<i32>(value | mask);
    }
    return static_cast<i32>(value);
}

Decoded decode(u32 raw) {
    Decoded d;
    d.raw = raw;

    // Extract standard RISC-V fields
    d.opcode = raw & 0x7F;
    d.rd     = (raw >> 7)  & 0x1F;
    d.funct3 = (raw >> 12) & 0x7;
    d.rs1    = (raw >> 15) & 0x1F;
    d.rs2    = (raw >> 20) & 0x1F;
    d.funct7 = (raw >> 25) & 0x7F;

    // Decode based on opcode
    switch (d.opcode) {
        case 0x33: // R-type integer
            if (d.funct3 == 0x0 && d.funct7 == 0x00) d.kind = Decoded::Kind::ADD;
            else if (d.funct3 == 0x0 && d.funct7 == 0x20) d.kind = Decoded::Kind::SUB;
            else if (d.funct3 == 0x7 && d.funct7 == 0x00) d.kind = Decoded::Kind::AND;
            else if (d.funct3 == 0x6 && d.funct7 == 0x00) d.kind = Decoded::Kind::OR;
            else if (d.funct3 == 0x4 && d.funct7 == 0x00) d.kind = Decoded::Kind::XOR;
            else if (d.funct3 == 0x1 && d.funct7 == 0x00) d.kind = Decoded::Kind::SLLI;
            else if (d.funct3 == 0x5 && d.funct7 == 0x00) d.kind = Decoded::Kind::SRLI;
            else if (d.funct3 == 0x5 && d.funct7 == 0x20) d.kind = Decoded::Kind::SRAI;
            else if (d.funct3 == 0x2 && d.funct7 == 0x00) d.kind = Decoded::Kind::SLT;
            else if (d.funct3 == 0x3 && d.funct7 == 0x00) d.kind = Decoded::Kind::SLTU;
            d.imm = 0;
            break;

        case 0x13: // I-type arithmetic
            if (d.funct3 == 0x0) d.kind = Decoded::Kind::ADDI;
            else if (d.funct3 == 0x7) d.kind = Decoded::Kind::ANDI;
            else if (d.funct3 == 0x6) d.kind = Decoded::Kind::ORI;
            else if (d.funct3 == 0x4) d.kind = Decoded::Kind::XORI;
            else if (d.funct3 == 0x1) d.kind = Decoded::Kind::SLLI;
            else if (d.funct3 == 0x5 && ((raw >> 25) & 0x7F) == 0x00) d.kind = Decoded::Kind::SRLI;
            else if (d.funct3 == 0x5 && ((raw >> 25) & 0x7F) == 0x20) d.kind = Decoded::Kind::SRAI;
            else if (d.funct3 == 0x2) d.kind = Decoded::Kind::SLTI;
            else if (d.funct3 == 0x3) d.kind = Decoded::Kind::SLTIU;
            d.imm = sign_extend((raw >> 20) & 0xFFF, 12);
            break;

        case 0x03: // Load
            if (d.funct3 == 0x2) d.kind = Decoded::Kind::LW;
            d.imm = sign_extend((raw >> 20) & 0xFFF, 12);
            break;

        case 0x23: // Store
            if (d.funct3 == 0x2) d.kind = Decoded::Kind::SW;
            d.imm = sign_extend(((raw >> 25) << 5) | ((raw >> 7) & 0x1F), 12);
            break;

        case 0x07: // FP Load
            if (d.funct3 == 0x2) d.kind = Decoded::Kind::FLW;
            d.imm = sign_extend((raw >> 20) & 0xFFF, 12);
            break;

        case 0x27: // FP Store
            if (d.funct3 == 0x2) d.kind = Decoded::Kind::FSW;
            d.imm = sign_extend(((raw >> 25) << 5) | ((raw >> 7) & 0x1F), 12);
            break;

        case 0x53: // FP operations
            {
                u8 funct7_fp = (raw >> 25) & 0x7F;
                if (funct7_fp == 0x00) d.kind = Decoded::Kind::FADD_S;
                else if (funct7_fp == 0x04) d.kind = Decoded::Kind::FSUB_S;
                else if (funct7_fp == 0x08) d.kind = Decoded::Kind::FMUL_S;
                else if (funct7_fp == 0x0C) d.kind = Decoded::Kind::FDIV_S;
                d.imm = 0;
            }
            break;

        case 0x63: // Branch
            if (d.funct3 == 0x0) d.kind = Decoded::Kind::BEQ;
            else if (d.funct3 == 0x1) d.kind = Decoded::Kind::BNE;
            else if (d.funct3 == 0x4) d.kind = Decoded::Kind::BLT;
            else if (d.funct3 == 0x5) d.kind = Decoded::Kind::BGE;
            else if (d.funct3 == 0x6) d.kind = Decoded::Kind::BLTU;
            else if (d.funct3 == 0x7) d.kind = Decoded::Kind::BGEU;
            d.imm = sign_extend(((raw >> 31) << 12) | (((raw >> 7) & 0x1) << 11) |
                                (((raw >> 25) & 0x3F) << 5) | (((raw >> 8) & 0xF) << 1), 13);
            break;

        case 0x6F: // JAL
            d.kind = Decoded::Kind::JAL;
            d.imm = sign_extend(((raw >> 31) << 20) | (((raw >> 12) & 0xFF) << 12) |
                                (((raw >> 20) & 0x1) << 11) | (((raw >> 21) & 0x3FF) << 1), 21);
            break;

        case 0x67: // JALR
            if (d.funct3 == 0x0) d.kind = Decoded::Kind::JALR;
            d.imm = sign_extend((raw >> 20) & 0xFFF, 12);
            break;

        case 0x37: // LUI
            d.kind = Decoded::Kind::LUI;
            d.imm = static_cast<i32>(raw & 0xFFFFF000);
            break;

        case 0x17: // AUIPC
            d.kind = Decoded::Kind::AUIPC;
            d.imm = static_cast<i32>(raw & 0xFFFFF000);
            break;

        default:
            if (raw == 0x00000013) { // ADDI x0, x0, 0 (canonical NOP)
                d.kind = Decoded::Kind::NOP;
            } else {
                d.kind = Decoded::Kind::Invalid;
            }
            break;
    }

    return d;
}

// Integer ALU operations
u32 exec_add(u32 a, u32 b) { return a + b; }
u32 exec_sub(u32 a, u32 b) { return a - b; }
u32 exec_and(u32 a, u32 b) { return a & b; }
u32 exec_or(u32 a, u32 b) { return a | b; }
u32 exec_xor(u32 a, u32 b) { return a ^ b; }
u32 exec_sll(u32 a, u32 shamt) { return a << (shamt & 0x1F); }
u32 exec_srl(u32 a, u32 shamt) { return a >> (shamt & 0x1F); }
u32 exec_sra(i32 a, u32 shamt) { return static_cast<u32>(a >> (shamt & 0x1F)); }
u32 exec_slt(i32 a, i32 b) { return (a < b) ? 1 : 0; }
u32 exec_sltu(u32 a, u32 b) { return (a < b) ? 1 : 0; }

// Floating-point operations
f32 exec_fadd_s(f32 a, f32 b) { return a + b; }
f32 exec_fsub_s(f32 a, f32 b) { return a - b; }
f32 exec_fmul_s(f32 a, f32 b) { return a * b; }
f32 exec_fdiv_s(f32 a, f32 b) { return a / b; }

// Branch conditions
bool eval_beq(u32 a, u32 b) { return a == b; }
bool eval_bne(u32 a, u32 b) { return a != b; }
bool eval_blt(i32 a, i32 b) { return a < b; }
bool eval_bge(i32 a, i32 b) { return a >= b; }
bool eval_bltu(u32 a, u32 b) { return a < b; }
bool eval_bgeu(u32 a, u32 b) { return a >= b; }

// Address calculation
u32 calc_branch_target(u32 pc, i32 offset) { return pc + static_cast<u32>(offset); }
u32 calc_jump_target(u32 pc, i32 offset) { return pc + static_cast<u32>(offset); }
u32 calc_jalr_target(u32 base, i32 offset) { return (base + static_cast<u32>(offset)) & ~1u; }

} // namespace isa
