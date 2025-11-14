#include "sim/include/memory.hpp"
#include "sim/include/loader.hpp"
#include <iostream>
#include <iomanip>

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

    loader::load_program_vsub(mem, map);

    std::cout << "First 40 instructions at address 0x100 (vsub):\n";
    for (int i = 0; i < 40; i++) {
        u32 addr = 0x100 + i * 4;
        u32 inst = mem.ifetch(addr);
        std::cout << "0x" << std::hex << std::setw(3) << std::setfill('0') << addr
                  << ": 0x" << std::setw(8) << std::setfill('0') << inst << "\n";
    }

    return 0;
}
