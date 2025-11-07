#pragma once
#include "types.hpp"
#include "memory.hpp"
#include <vector>

// Memory map configuration
struct MemMap {
    u32 inst_base = 0x000;
    u32 inst_size = 0x100;
    u32 stack_base = 0x200;
    u32 stack_size = 0x100;
    u32 A_base = 0x400;
    u32 B_base = 0x800;
    u32 C_base = 0xC00;
    u32 D_base = 0x1000;
    int length = 256;  // Vector length
};

namespace loader {
    // Initialize memory with vadd program (CPU0)
    void load_program_vadd(Memory& mem, const MemMap& map);

    // Initialize memory with vsub program (CPU1)
    void load_program_vsub(Memory& mem, const MemMap& map);

    // Initialize arrays A and B with random FP32 values
    void init_arrays(Memory& mem, const MemMap& map, unsigned seed = 1);

    // Clear result arrays C and D
    void clear_results(Memory& mem, const MemMap& map);

    // Helper: encode an instruction manually
    u32 encode_r_type(u8 opcode, u8 rd, u8 funct3, u8 rs1, u8 rs2, u8 funct7);
    u32 encode_i_type(u8 opcode, u8 rd, u8 funct3, u8 rs1, i32 imm);
    u32 encode_s_type(u8 opcode, u8 funct3, u8 rs1, u8 rs2, i32 imm);
    u32 encode_b_type(u8 opcode, u8 funct3, u8 rs1, u8 rs2, i32 imm);
    u32 encode_u_type(u8 opcode, u8 rd, u32 imm);
    u32 encode_j_type(u8 opcode, u8 rd, i32 imm);
}
