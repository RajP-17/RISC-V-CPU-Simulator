#pragma once
#include <cstdint>
#include <array>

// Type aliases for clarity
using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using i32 = std::int32_t;
using i64 = std::int64_t;
using f32 = float;

// Decoded instruction representation
struct Decoded {
    u32 raw{};          // Raw 32-bit instruction

    // Instruction fields
    u8 opcode{};
    u8 rd{};
    u8 rs1{};
    u8 rs2{};
    u8 funct3{};
    u8 funct7{};
    i32 imm{};

    // Instruction type classification
    enum class Kind {
        // Integer instructions
        ADD, SUB, ADDI,
        LUI, AUIPC,
        SLLI, SRLI, SRAI,
        AND, OR, XOR, ANDI, ORI, XORI,
        SLT, SLTI, SLTU, SLTIU,

        // Memory operations
        LW, SW,

        // Floating-point instructions
        FLW, FSW,
        FADD_S, FSUB_S, FMUL_S, FDIV_S,

        // Control flow
        JAL, JALR,
        BEQ, BNE, BLT, BGE, BLTU, BGEU,

        // Special
        NOP,
        Invalid
    } kind{Kind::Invalid};

    // Is this a floating-point instruction?
    bool is_fp() const {
        return kind == Kind::FLW || kind == Kind::FSW ||
               kind == Kind::FADD_S || kind == Kind::FSUB_S ||
               kind == Kind::FMUL_S || kind == Kind::FDIV_S;
    }

    // Is this a memory instruction?
    bool is_memory() const {
        return kind == Kind::LW || kind == Kind::SW ||
               kind == Kind::FLW || kind == Kind::FSW;
    }

    // Is this a branch instruction?
    bool is_branch() const {
        return kind == Kind::BEQ || kind == Kind::BNE ||
               kind == Kind::BLT || kind == Kind::BGE ||
               kind == Kind::BLTU || kind == Kind::BGEU;
    }

    // Is this a jump instruction?
    bool is_jump() const {
        return kind == Kind::JAL || kind == Kind::JALR;
    }
};
