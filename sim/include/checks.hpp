#pragma once
#include "types.hpp"
#include "memory.hpp"
#include "loader.hpp"
#include <cmath>

namespace checks {
    // Host-side reference computation for validation
    void host_vadd(const f32* A, const f32* B, f32* C, int n);
    void host_vsub(const f32* A, const f32* B, f32* D, int n);

    // Floating-point comparison with ULP tolerance
    bool approximately_equal(f32 a, f32 b, int ulp = 1);

    // Validate simulation results
    bool validate_vadd(const Memory& mem, const MemMap& map, int n);
    bool validate_vsub(const Memory& mem, const MemMap& map, int n);
}
