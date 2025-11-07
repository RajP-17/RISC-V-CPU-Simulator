#pragma once
#include "types.hpp"
#include <cmath>

namespace isa {
    // Decode a 32-bit RISC-V instruction
    Decoded decode(u32 raw);

    // Sign-extend immediate values
    i32 sign_extend(u32 value, int bits);

    // Execute primitives (these modify CPU state through references)

    // Integer ALU operations
    u32 exec_add(u32 a, u32 b);
    u32 exec_sub(u32 a, u32 b);
    u32 exec_and(u32 a, u32 b);
    u32 exec_or(u32 a, u32 b);
    u32 exec_xor(u32 a, u32 b);
    u32 exec_sll(u32 a, u32 shamt);
    u32 exec_srl(u32 a, u32 shamt);
    u32 exec_sra(i32 a, u32 shamt);
    u32 exec_slt(i32 a, i32 b);
    u32 exec_sltu(u32 a, u32 b);

    // Floating-point ALU operations
    f32 exec_fadd_s(f32 a, f32 b);
    f32 exec_fsub_s(f32 a, f32 b);
    f32 exec_fmul_s(f32 a, f32 b);
    f32 exec_fdiv_s(f32 a, f32 b);

    // Branch condition evaluation
    bool eval_beq(u32 a, u32 b);
    bool eval_bne(u32 a, u32 b);
    bool eval_blt(i32 a, i32 b);
    bool eval_bge(i32 a, i32 b);
    bool eval_bltu(u32 a, u32 b);
    bool eval_bgeu(u32 a, u32 b);

    // Address calculation
    u32 calc_branch_target(u32 pc, i32 offset);
    u32 calc_jump_target(u32 pc, i32 offset);
    u32 calc_jalr_target(u32 base, i32 offset);
}
