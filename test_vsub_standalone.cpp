#include "sim/include/cpu.hpp"
#include "sim/include/memory.hpp"
#include "sim/include/loader.hpp"
#include <iostream>

int main() {
    Memory mem(64 * 1024);
    MemMap map;
    map.inst_base = 0x000;
    map.inst_size = 0x100;
    map.stack_base = 0x200;
    map.stack_size = 0x100;
    map.A_base = 0x400;
    map.B_base = 0x800;
    map.C_base = 0xC00;
    map.D_base = 0x1000;
    map.length = 256;

    // Load BOTH programs, just like main_dual_simple
    loader::load_program_vadd(mem, map);  // vadd at 0x000
    loader::load_program_vsub(mem, map);  // vsub at 0x100
    loader::init_arrays(mem, map, 1);
    loader::clear_results(mem, map);

    // Run CPU0 first (vadd)
    std::cout << "Running CPU0 (vadd)...\n";
    Cpu cpu0(mem, 0);
    cpu0.reset(0x000);
    Stats stats0 = cpu0.run(0);
    std::cout << "CPU0 complete: " << stats0.retired << " instructions\n\n";

    // Now run CPU1 (vsub) - THIS IS WHAT FAILS IN MAIN_DUAL_SIMPLE
    std::cout << "Running CPU1 (vsub)...\n";
    Cpu cpu1(mem, 1);
    cpu1.reset(0x100);

    // Run with timeout of 1 million ticks
    for (int i = 0; i < 1000000; i++) {
        cpu1.tick();

        // Print loop counter every 10000 ticks
        if (i % 10000 == 0) {
            u32 sp = cpu1.get_ireg(2);  // sp
            u32 s0 = cpu1.get_ireg(8);  // s0/fp
            std::cout << "Tick " << i << ": PC=0x" << std::hex << cpu1.get_pc()
                      << " sp=0x" << sp << " s0=0x" << s0 << std::dec
                      << " retired=" << cpu1.stats().retired << "\n";
        }

        if (cpu1.halted()) {
            std::cout << "CPU1 halted after " << cpu1.stats().retired << " instructions\n";
            break;
        }
    }

    if (!cpu1.halted()) {
        std::cout << "CPU1 did NOT halt after 1000000 ticks\n";
        std::cout << "Final PC: 0x" << std::hex << cpu1.get_pc() << std::dec << "\n";
        std::cout << "Retired: " << cpu1.stats().retired << "\n";
    }

    return 0;
}
