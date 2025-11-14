#include "cpu.hpp"
#include "memory.hpp"
#include "loader.hpp"
#include "checks.hpp"
#include "bus.hpp"
#include "mem_adapter.hpp"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    // Parse command line arguments
    u64 max_ticks = 0;
    if (argc > 2 && std::string(argv[1]) == "--ticks") {
        max_ticks = std::stoull(argv[2]);
    }

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

    // Create shared memory
    Memory mem(64 * 1024);  // 64KB

    // Load programs
    loader::load_program_vadd(mem, map);  // CPU0 at 0x000
    loader::load_program_vsub(mem, map);  // CPU1 at 0x100

    // Initialize arrays A and B
    loader::init_arrays(mem, map, 1);  // Fixed seed for reproducibility
    loader::clear_results(mem, map);

    // Create bus arbiter
    BusArbiter bus(mem);

    // Create memory adapters for each CPU
    MemoryAdapter cpu0_mem(bus.memory(), bus.cpu0_dport());
    MemoryAdapter cpu1_mem(bus.memory(), bus.cpu1_dport());

    // Create CPUs (MemoryAdapter inherits from Memory, so no cast needed)
    Cpu cpu0(cpu0_mem, 0);
    Cpu cpu1(cpu1_mem, 1);

    // Reset CPUs to their starting addresses
    cpu0.reset(0x000);  // vadd starts at 0x000
    cpu1.reset(0x100);  // vsub starts at 0x100

    // Run dual-CPU simulation
    std::cout << "=== Dual-CPU Simulation (Part 2) ===\n";
    std::cout << "CPU0: vadd (C = A + B) starting at 0x000\n";
    std::cout << "CPU1: vsub (D = A - B) starting at 0x100\n";
    std::cout << "----------------------------------------\n";

    u64 ticks = 0;
    while ((!cpu0.halted() || !cpu1.halted()) && (max_ticks == 0 || ticks < max_ticks)) {
        // Tick both CPUs
        if (!cpu0.halted()) cpu0.tick();
        if (!cpu1.halted()) cpu1.tick();

        // Tick the bus arbiter (handles contention)
        bus.tick();

        ticks++;

        // Progress indicator every 50000 ticks
        if (ticks % 50000 == 0) {
            std::cout << "Progress: " << ticks << " ticks, CPU0: " << cpu0.stats().retired
                      << " retired, CPU1: " << cpu1.stats().retired << " retired\n";
        }
    }

    // Get final statistics
    const Stats& stats0 = cpu0.stats();
    const Stats& stats1 = cpu1.stats();

    std::cout << "\n=== CPU0 Statistics (vadd) ===\n";
    std::cout << "Instructions retired: " << stats0.retired << "\n";
    std::cout << "Total cycles: " << stats0.cycles << "\n";
    std::cout << "Total ticks: " << stats0.ticks << "\n";
    std::cout << "CPI: " << std::fixed << std::setprecision(3) << stats0.cpi() << "\n";
    std::cout << "Loads: " << stats0.loads << "\n";
    std::cout << "Stores: " << stats0.stores << "\n";
    std::cout << "Stalls: " << stats0.stalls << "\n";

    std::cout << "\n=== CPU1 Statistics (vsub) ===\n";
    std::cout << "Instructions retired: " << stats1.retired << "\n";
    std::cout << "Total cycles: " << stats1.cycles << "\n";
    std::cout << "Total ticks: " << stats1.ticks << "\n";
    std::cout << "CPI: " << std::fixed << std::setprecision(3) << stats1.cpi() << "\n";
    std::cout << "Loads: " << stats1.loads << "\n";
    std::cout << "Stores: " << stats1.stores << "\n";
    std::cout << "Stalls: " << stats1.stalls << "\n";

    std::cout << "\n=== Overall Statistics ===\n";
    std::cout << "Total simulation ticks: " << ticks << "\n";
    std::cout << "Total cycles (both CPUs): " << (stats0.cycles + stats1.cycles) << "\n";
    std::cout << "Total instructions retired: " << (stats0.retired + stats1.retired) << "\n";

    // Validate results
    std::cout << "\n=== Validation ===\n";
    bool vadd_ok = checks::validate_vadd(mem, map, map.length);
    bool vsub_ok = checks::validate_vsub(mem, map, map.length);

    if (vadd_ok) {
        std::cout << "✓ vadd validation PASSED - Results are correct!\n";
    } else {
        std::cerr << "✗ vadd validation FAILED!\n";
    }

    if (vsub_ok) {
        std::cout << "✓ vsub validation PASSED - Results are correct!\n";
    } else {
        std::cerr << "✗ vsub validation FAILED!\n";
    }

    return (vadd_ok && vsub_ok) ? 0 : 1;
}
