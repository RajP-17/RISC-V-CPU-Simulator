#include "loader.hpp"
#include <random>
#include <cstring>
#include <iostream>
#include <iomanip>

namespace loader {

u32 encode_r_type(u8 opcode, u8 rd, u8 funct3, u8 rs1, u8 rs2, u8 funct7) {
    return opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (funct7 << 25);
}

u32 encode_i_type(u8 opcode, u8 rd, u8 funct3, u8 rs1, i32 imm) {
    u32 imm_bits = (static_cast<u32>(imm) & 0xFFF) << 20;
    return opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | imm_bits;
}

u32 encode_s_type(u8 opcode, u8 funct3, u8 rs1, u8 rs2, i32 imm) {
    u32 imm_low = (static_cast<u32>(imm) & 0x1F) << 7;
    u32 imm_high = ((static_cast<u32>(imm) >> 5) & 0x7F) << 25;
    return opcode | imm_low | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | imm_high;
}

u32 encode_b_type(u8 opcode, u8 funct3, u8 rs1, u8 rs2, i32 imm) {
    u32 imm11 = ((static_cast<u32>(imm) >> 11) & 0x1) << 7;
    u32 imm_4_1 = ((static_cast<u32>(imm) >> 1) & 0xF) << 8;
    u32 imm_10_5 = ((static_cast<u32>(imm) >> 5) & 0x3F) << 25;
    u32 imm12 = ((static_cast<u32>(imm) >> 12) & 0x1) << 31;
    return opcode | imm11 | imm_4_1 | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | imm_10_5 | imm12;
}

u32 encode_u_type(u8 opcode, u8 rd, u32 imm) {
    return opcode | (rd << 7) | (imm & 0xFFFFF000);
}

u32 encode_j_type(u8 opcode, u8 rd, i32 imm) {
    // J-type immediate encoding: imm[20|10:1|11|19:12]
    // Instruction bits: [31]=imm[20], [30:21]=imm[10:1], [20]=imm[11], [19:12]=imm[19:12]
    u32 imm_19_12 = ((static_cast<u32>(imm) >> 12) & 0xFF) << 12;  // bits [19:12]
    u32 imm11 = ((static_cast<u32>(imm) >> 11) & 0x1) << 20;        // bit [20]
    u32 imm_10_1 = ((static_cast<u32>(imm) >> 1) & 0x3FF) << 21;    // bits [30:21]
    u32 imm20 = ((static_cast<u32>(imm) >> 20) & 0x1) << 31;        // bit [31]
    return opcode | (rd << 7) | imm_19_12 | imm11 | imm_10_1 | imm20;
}

void load_program_vadd(Memory& mem, const MemMap& map) {
    // Based on CPU0.s assembly code
    // This is a simplified loop: for(i=0; i<256; i++) C[i] = A[i] + B[i]

    u32 addr = map.inst_base;
    std::vector<u32> program;

    // Prologue
    program.push_back(encode_i_type(0x13, 2, 0x0, 2, -16));  // addi sp, sp, -16
    program.push_back(encode_s_type(0x23, 0x2, 2, 1, 12));   // sw ra, 12(sp)
    program.push_back(encode_s_type(0x23, 0x2, 2, 8, 8));    // sw s0, 8(sp)
    program.push_back(encode_i_type(0x13, 8, 0x0, 2, 16));   // addi s0, sp, 16

    // Initialize return value and loop counter i = 0
    program.push_back(encode_i_type(0x13, 10, 0x0, 0, 0));   // addi a0, x0, 0  (mv a0, zero)
    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -12)); // sw a0, -12(s0) [return value = 0]
    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -16)); // sw a0, -16(s0) [i=0]

    // Loop start (j .LBB0_1)
    u32 loop_check_start = program.size();  // Mark start of .LBB0_1
    program.push_back(encode_j_type(0x6F, 0, 4));  // j .LBB0_1

    // .LBB0_1: Loop condition check
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -16)); // lw a0, -16(s0)
    program.push_back(encode_i_type(0x13, 11, 0x0, 0, 255)); // addi a1, zero, 255
    u32 blt_pos = program.size();
    program.push_back(0);  // Placeholder for blt - will fix later
    program.push_back(encode_j_type(0x6F, 0, 4));             // j .LBB0_2

    // .LBB0_2: Loop body
    // Load A[i] - properly handle %hi/%lo split
    u32 A_hi = map.A_base & 0xFFFFF000;
    i32 A_lo = static_cast<i32>(map.A_base & 0xFFF);
    if (A_lo & 0x800) A_hi += 0x1000;  // Adjust for sign extension
    program.push_back(encode_u_type(0x37, 10, A_hi)); // lui a0, %hi(A_base)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, A_lo)); // addi a0, a0, %lo(A_base)
    program.push_back(encode_i_type(0x03, 11, 0x2, 8, -16)); // lw a1, -16(s0) [i]
    program.push_back(encode_i_type(0x13, 11, 0x1, 11, 2)); // slli a1, a1, 2
    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00)); // add a0, a0, a1
    program.push_back(encode_i_type(0x07, 0, 0x2, 10, 0)); // flw ft0, 0(a0)

    // Load B[i] - properly handle %hi/%lo split
    u32 B_hi = map.B_base & 0xFFFFF000;
    i32 B_lo = static_cast<i32>(map.B_base & 0xFFF);
    if (B_lo & 0x800) B_hi += 0x1000;  // Adjust for sign extension
    program.push_back(encode_u_type(0x37, 10, B_hi)); // lui a0, %hi(B_base)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, B_lo)); // addi a0, a0, %lo(B_base)
    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00)); // add a0, a0, a1
    program.push_back(encode_i_type(0x07, 1, 0x2, 10, 0)); // flw ft1, 0(a0)

    // C[i] = A[i] + B[i]
    program.push_back(encode_r_type(0x53, 0, 0x0, 0, 1, 0x00)); // fadd.s ft0, ft0, ft1

    // Store C[i] - properly handle %hi/%lo split
    u32 C_hi = map.C_base & 0xFFFFF000;
    i32 C_lo = static_cast<i32>(map.C_base & 0xFFF);
    if (C_lo & 0x800) C_hi += 0x1000;  // Adjust for sign extension
    program.push_back(encode_u_type(0x37, 10, C_hi)); // lui a0, %hi(C_base)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, C_lo)); // addi a0, a0, %lo(C_base)
    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00)); // add a0, a0, a1
    program.push_back(encode_s_type(0x27, 0x2, 10, 0, 0)); // fsw ft0, 0(a0)

    program.push_back(encode_j_type(0x6F, 0, 4)); // j .LBB0_3

    // .LBB0_3: Increment i
    u32 increment_start = program.size();
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -16)); // lw a0, -16(s0)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, 1)); // addi a0, a0, 1
    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -16)); // sw a0, -16(s0)
    // Calculate jump offset back to .LBB0_1
    u32 jump_from = program.size();  // Position of the jump instruction
    u32 jump_target = loop_check_start + 1;  // Target is .LBB0_1 (after the first j instruction)
    i32 jump_offset = (static_cast<i32>(jump_target) - static_cast<i32>(jump_from)) * 4;
    u32 encoded_jal = encode_j_type(0x6F, 0, jump_offset);
    program.push_back(encoded_jal); // j .LBB0_1 (back to loop start)

    // .LBB0_4: Epilogue
    u32 epilogue_start = program.size();
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -12)); // lw a0, -12(s0)
    program.push_back(encode_i_type(0x03, 8, 0x2, 2, 8));    // lw s0, 8(sp)
    program.push_back(encode_i_type(0x03, 1, 0x2, 2, 12));   // lw ra, 12(sp)
    program.push_back(encode_i_type(0x13, 2, 0x0, 2, 16));   // addi sp, sp, 16
    program.push_back(0x00008067);  // ret (jalr x0, 0(x1))

    // Now fix the BLT branch offset to jump to epilogue
    i32 blt_offset = (epilogue_start - blt_pos) * 4;
    program[blt_pos] = encode_b_type(0x63, 0x4, 11, 10, blt_offset);  // blt a1, a0, .LBB0_4

    // Write program to memory
    for (size_t i = 0; i < program.size(); ++i) {
        mem.write_u32(addr + i * 4, program[i]);
    }
}

void load_program_vsub(Memory& mem, const MemMap& map) {
    // Similar to vadd but with FSUB and different result array (D instead of C)
    // Starting at address 0x100
    u32 addr = 0x100;
    std::vector<u32> program;

    // Prologue - EXACTLY like vadd
    program.push_back(encode_i_type(0x13, 2, 0x0, 2, -16));  // addi sp, sp, -16
    program.push_back(encode_s_type(0x23, 0x2, 2, 1, 12));   // sw ra, 12(sp)
    program.push_back(encode_s_type(0x23, 0x2, 2, 8, 8));    // sw s0, 8(sp)
    program.push_back(encode_i_type(0x13, 8, 0x0, 2, 16));   // addi s0, sp, 16

    // Initialize return value and loop counter i = 0
    program.push_back(encode_i_type(0x13, 10, 0x0, 0, 0));   // addi a0, x0, 0  (mv a0, zero)
    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -12)); // sw a0, -12(s0) [return value = 0]
    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -16)); // sw a0, -16(s0) [i=0]

    // Loop start (j .LBB0_1)
    u32 loop_check_start = program.size();  // Mark start of .LBB0_1
    program.push_back(encode_j_type(0x6F, 0, 4));  // j .LBB0_1

    // .LBB0_1: Loop condition check
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -16)); // lw a0, -16(s0)
    program.push_back(encode_i_type(0x13, 11, 0x0, 0, 255)); // addi a1, zero, 255
    u32 blt_pos = program.size();
    program.push_back(0);  // Placeholder for blt - will fix later
    program.push_back(encode_j_type(0x6F, 0, 4));             // j .LBB0_2

    // .LBB0_2: Loop body
    // Load A[i] - properly handle %hi/%lo split
    u32 A_hi = map.A_base & 0xFFFFF000;
    i32 A_lo = static_cast<i32>(map.A_base & 0xFFF);
    if (A_lo & 0x800) A_hi += 0x1000;  // Adjust for sign extension
    program.push_back(encode_u_type(0x37, 10, A_hi)); // lui a0, %hi(A_base)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, A_lo)); // addi a0, a0, %lo(A_base)
    program.push_back(encode_i_type(0x03, 11, 0x2, 8, -16)); // lw a1, -16(s0) [i]
    program.push_back(encode_i_type(0x13, 11, 0x1, 11, 2)); // slli a1, a1, 2
    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00)); // add a0, a0, a1
    program.push_back(encode_i_type(0x07, 0, 0x2, 10, 0)); // flw ft0, 0(a0)

    // Load B[i] - properly handle %hi/%lo split
    u32 B_hi = map.B_base & 0xFFFFF000;
    i32 B_lo = static_cast<i32>(map.B_base & 0xFFF);
    if (B_lo & 0x800) B_hi += 0x1000;  // Adjust for sign extension
    program.push_back(encode_u_type(0x37, 10, B_hi)); // lui a0, %hi(B_base)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, B_lo)); // addi a0, a0, %lo(B_base)
    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00)); // add a0, a0, a1
    program.push_back(encode_i_type(0x07, 1, 0x2, 10, 0)); // flw ft1, 0(a0)

    // D[i] = A[i] - B[i] (FSUB instead of FADD)
    program.push_back(encode_r_type(0x53, 0, 0x0, 0, 1, 0x04)); // fsub.s ft0, ft0, ft1

    // Store D[i] - properly handle %hi/%lo split
    u32 D_hi = map.D_base & 0xFFFFF000;
    i32 D_lo = static_cast<i32>(map.D_base & 0xFFF);
    if (D_lo & 0x800) D_hi += 0x1000;  // Adjust for sign extension
    program.push_back(encode_u_type(0x37, 10, D_hi)); // lui a0, %hi(D_base)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, D_lo)); // addi a0, a0, %lo(D_base)
    program.push_back(encode_r_type(0x33, 10, 0x0, 10, 11, 0x00)); // add a0, a0, a1
    program.push_back(encode_s_type(0x27, 0x2, 10, 0, 0)); // fsw ft0, 0(a0)

    program.push_back(encode_j_type(0x6F, 0, 4)); // j .LBB0_3

    // .LBB0_3: Increment i
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -16)); // lw a0, -16(s0)
    program.push_back(encode_i_type(0x13, 10, 0x0, 10, 1)); // addi a0, a0, 1
    program.push_back(encode_s_type(0x23, 0x2, 8, 10, -16)); // sw a0, -16(s0)
    // Calculate jump offset back to .LBB0_1
    u32 jump_from = program.size();  // Position of the jump instruction
    u32 jump_target = loop_check_start + 1;  // Target is .LBB0_1 (after the first j instruction)
    i32 jump_offset = (static_cast<i32>(jump_target) - static_cast<i32>(jump_from)) * 4;
    program.push_back(encode_j_type(0x6F, 0, jump_offset)); // j .LBB0_1 (back to loop start)

    // .LBB0_4: Epilogue
    u32 epilogue_start = program.size();
    program.push_back(encode_i_type(0x03, 10, 0x2, 8, -12)); // lw a0, -12(s0)
    program.push_back(encode_i_type(0x03, 8, 0x2, 2, 8));    // lw s0, 8(sp)
    program.push_back(encode_i_type(0x03, 1, 0x2, 2, 12));   // lw ra, 12(sp)
    program.push_back(encode_i_type(0x13, 2, 0x0, 2, 16));   // addi sp, sp, 16
    program.push_back(0x00008067);  // ret (jalr x0, 0(x1))

    // Now fix the BLT branch offset to jump to epilogue
    i32 blt_offset = (epilogue_start - blt_pos) * 4;
    program[blt_pos] = encode_b_type(0x63, 0x4, 11, 10, blt_offset);  // blt a1, a0, .LBB0_4

    // Write program to memory
    for (size_t i = 0; i < program.size(); ++i) {
        mem.write_u32(addr + i * 4, program[i]);
    }
}

void init_arrays(Memory& mem, const MemMap& map, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<f32> dist(-100.0f, 100.0f);

    // Initialize A and B arrays
    for (int i = 0; i < map.length; ++i) {
        f32 a_val = dist(rng);
        f32 b_val = dist(rng);

        mem.write_f32(map.A_base + i * 4, a_val);
        mem.write_f32(map.B_base + i * 4, b_val);
    }
}

void clear_results(Memory& mem, const MemMap& map) {
    for (int i = 0; i < map.length; ++i) {
        mem.write_f32(map.C_base + i * 4, 0.0f);
        mem.write_f32(map.D_base + i * 4, 0.0f);
    }
}

} // namespace loader
