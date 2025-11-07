#include "cpu.hpp"
#include "memory.hpp"
#include "loader.hpp"
#include "checks.hpp"
#include "timing.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>

int main(int argc, char** argv) {
    // Configuration
    int n = 256;          // Vector length
    unsigned seed = 1;     // Random seed
    u64 max_ticks = 0;     // 0 = run until halt

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--ticks" && i + 1 < argc) {
            max_ticks = std::stoull(argv[++i]);
        } else if (arg == "--len" && i + 1 < argc) {
            n = std::stoi(argv[++i]);
        } else if (arg == "--seed" && i + 1 < argc) {
            seed = std::stoul(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --ticks N    Run for N ticks (0 = until halt)\n";
            std::cout << "  --len N      Vector length (default: 256)\n";
            std::cout << "  --seed N     Random seed (default: 1)\n";
            std::cout << "  --help       Show this help\n";
            return 0;
        }
    }

    std::cout << "========================================\n";
    std::cout << "  RISC-V Pipelined CPU Simulator\n";
    std::cout << "  Part 1: Single CPU (vadd)\n";
    std::cout << "========================================\n\n";

    // Create memory map
    MemMap map;
    map.inst_base = 0x000;
    map.inst_size = 0x100;
    map.stack_base = 0x200;
    map.stack_size = 0x100;
    map.A_base = 0x400;
    map.B_base = 0x800;
    map.C_base = 0xC00;
    map.D_base = 0x1000;
    map.length = n;

    // Initialize memory
    Memory mem(64 * 1024);  // 64KB RAM

    std::cout << "Loading program...\n";
    loader::load_program_vadd(mem, map);

    std::cout << "Initializing arrays (seed=" << seed << ")...\n";
    loader::init_arrays(mem, map, seed);
    loader::clear_results(mem, map);

    // Debug: Print first 25 instructions in memory
    std::cout << "\nFirst 25 encoded instructions in memory:\n";
    for (int i = 0; i < 25; i++) {
        u32 addr = map.inst_base + i * 4;
        u32 instr = mem.read_u32(addr);
        std::cout << "  [0x" << std::hex << std::setw(3) << std::setfill('0') << addr
                  << "] = 0x" << std::setw(8) << instr << std::dec << "\n";
    }
    std::cout << "\n";

    // Create CPU
    Cpu cpu(mem, 0);
    cpu.reset(map.inst_base);

    std::cout << "Running simulation...\n";
    std::cout << "  Vector length: " << n << "\n";
    std::cout << "  Timing: " << timing::kTicksPerCycle << " ticks/cycle\n";
    std::cout << "  RV32I execute: " << timing::kIExecuteCycles << " cycle(s)\n";
    std::cout << "  RV32F execute: " << timing::kFExecuteCycles << " cycle(s)\n";
    std::cout << "  RAM latency: " << timing::kMemCycles << " cycle(s)\n\n";

    // Run simulation
    auto start_time = std::chrono::high_resolution_clock::now();
    Stats s = cpu.run(max_ticks);
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Simulation complete!\n\n";

    // Print statistics
    std::cout << "========================================\n";
    std::cout << "  Simulation Statistics\n";
    std::cout << "========================================\n";
    std::cout << "  Ticks:              " << s.ticks << "\n";
    std::cout << "  Cycles:             " << s.cycles << "\n";
    std::cout << "  Retired:            " << s.retired << " instructions\n";
    std::cout << "  Loads:              " << s.loads << "\n";
    std::cout << "  Stores:             " << s.stores << "\n";
    std::cout << "  Stalls:             " << s.stalls << "\n";
    std::cout << "  CPI:                " << std::fixed << std::setprecision(3) << s.cpi() << "\n";
    std::cout << "  Wall time:          " << duration.count() << " ms\n";
    std::cout << "========================================\n\n";

    // Validate results
    std::cout << "Validating results...\n";
    bool valid = checks::validate_vadd(mem, map, n);

    if (valid) {
        std::cout << "✓ Validation PASSED - Results are correct!\n";
    } else {
        std::cout << "✗ Validation FAILED - Results do not match reference!\n";
        return 1;
    }

    return 0;
}
