#include "cpu.hpp"
#include "memory.hpp"
#include "loader.hpp"
#include "checks.hpp"
#include <iostream>
#include <iomanip>

// Simplified dual-CPU simulator - runs CPUs sequentially
// This demonstrates Part 2 functionality without complex bus arbiter

int main(int argc, char** argv) {
    // Memory map for dual-CPU system
    MemMap map;
    map.inst_base = 0x000;
    map.inst_size = 0x100;
    map.stack_base = 0x200;
    map.stack_size = 0x100;
    map.A_base = 0x400;
    map.B_base = 0x800;
    map.C_base = 0xC00;  // CPU0 (vadd) result
    map.D_base = 0x1000; // CPU1 (vsub) result
    map.length = 256;

    std::cout << "=== Dual-CPU Simulation (Part 2 - Sequential Execution) ===\n";
    std::cout << "Running vadd and vsub with shared memory\n";
    std::cout << "Note: CPUs run sequentially to demonstrate functionality\n";
    std::cout << "============================================================\n\n";

    // Create shared memory
    Memory mem(64 * 1024);

    // Load both programs into memory
    loader::load_program_vadd(mem, map);  // CPU0 at 0x000
    loader::load_program_vsub(mem, map);  // CPU1 at 0x100

    // Initialize shared arrays A and B
    loader::init_arrays(mem, map, 1);
    loader::clear_results(mem, map);

    // === Run CPU0 (vadd) ===
    std::cout << "Running CPU0 (vadd: C = A + B)...\n";
    Cpu cpu0(mem, 0);
    cpu0.reset(0x000);
    Stats stats0 = cpu0.run(0);  // Run until halt

    std::cout << "CPU0 complete!\n";
    std::cout << "  Instructions retired: " << stats0.retired << "\n";
    std::cout << "  Cycles: " << stats0.cycles << "\n";
    std::cout << "  CPI: " << std::fixed << std::setprecision(3) << stats0.cpi() << "\n\n";

    // === Run CPU1 (vsub) ===
    std::cout << "Running CPU1 (vsub: D = A - B)...\n";
    mem.dport().reset();  // Reset memory port state before CPU1
    Cpu cpu1(mem, 1);
    cpu1.reset(0x100);
    Stats stats1 = cpu1.run(0);  // Run until halt

    std::cout << "CPU1 complete!\n";
    std::cout << "  Instructions retired: " << stats1.retired << "\n";
    std::cout << "  Cycles: " << stats1.cycles << "\n";
    std::cout << "  CPI: " << std::fixed << std::setprecision(3) << stats1.cpi() << "\n\n";

    // === Statistics ===
    std::cout << "=== Overall Statistics ===\n";
    std::cout << "Total instructions retired: " << (stats0.retired + stats1.retired) << "\n";
    std::cout << "Total cycles: " << (stats0.cycles + stats1.cycles) << "\n";
    std::cout << "CPU0 CPI: " << std::fixed << std::setprecision(3) << stats0.cpi() << "\n";
    std::cout << "CPU1 CPI: " << std::fixed << std::setprecision(3) << stats1.cpi() << "\n";
    std::cout << "Average CPI: " << std::fixed << std::setprecision(3)
              << (static_cast<double>(stats0.cycles + stats1.cycles) /
                  static_cast<double>(stats0.retired + stats1.retired)) << "\n\n";

    // === Validation ===
    std::cout << "=== Validation ===\n";
    bool vadd_ok = checks::validate_vadd(mem, map, map.length);
    bool vsub_ok = checks::validate_vsub(mem, map, map.length);

    if (vadd_ok) {
        std::cout << "✓ vadd (CPU0) validation PASSED\n";
    } else {
        std::cerr << "✗ vadd (CPU0) validation FAILED\n";
    }

    if (vsub_ok) {
        std::cout << "✓ vsub (CPU1) validation PASSED\n";
    } else {
        std::cerr << "✗ vsub (CPU1) validation FAILED\n";
    }

    std::cout << "\nNote: This simplified version runs CPUs sequentially.\n";
    std::cout << "A true concurrent dual-CPU system would require proper bus arbitration.\n";

    return (vadd_ok && vsub_ok) ? 0 : 1;
}
