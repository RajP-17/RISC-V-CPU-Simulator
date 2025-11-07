#include "checks.hpp"
#include <iostream>
#include <cstring>
#include <cmath>

namespace checks {

void host_vadd(const f32* A, const f32* B, f32* C, int n) {
    for (int i = 0; i < n; ++i) {
        C[i] = A[i] + B[i];
    }
}

void host_vsub(const f32* A, const f32* B, f32* D, int n) {
    for (int i = 0; i < n; ++i) {
        D[i] = A[i] - B[i];
    }
}

bool approximately_equal(f32 a, f32 b, int ulp) {
    // Check for exact equality first (handles infinities and zeros)
    if (a == b) return true;

    // Check for NaN
    if (std::isnan(a) || std::isnan(b)) return false;

    // Use relative epsilon for floating-point comparison
    f32 abs_diff = std::fabs(a - b);
    f32 larger = std::max(std::fabs(a), std::fabs(b));

    // Use ULP-based tolerance
    return abs_diff <= larger * ulp * std::numeric_limits<f32>::epsilon();
}

bool validate_vadd(const Memory& mem, const MemMap& map, int n) {
    // Read arrays from memory
    f32* A = new f32[n];
    f32* B = new f32[n];
    f32* C_sim = new f32[n];
    f32* C_ref = new f32[n];

    for (int i = 0; i < n; ++i) {
        A[i] = mem.read_f32(map.A_base + i * 4);
        B[i] = mem.read_f32(map.B_base + i * 4);
        C_sim[i] = mem.read_f32(map.C_base + i * 4);
    }

    // Compute reference
    host_vadd(A, B, C_ref, n);

    // Compare
    bool valid = true;
    for (int i = 0; i < n; ++i) {
        if (!approximately_equal(C_sim[i], C_ref[i], 2)) {
            std::cerr << "Mismatch at C[" << i << "]: "
                      << "sim=" << C_sim[i] << " ref=" << C_ref[i]
                      << " (A=" << A[i] << " B=" << B[i] << ")\n";
            valid = false;
            break;
        }
    }

    delete[] A;
    delete[] B;
    delete[] C_sim;
    delete[] C_ref;

    return valid;
}

bool validate_vsub(const Memory& mem, const MemMap& map, int n) {
    // Read arrays from memory
    f32* A = new f32[n];
    f32* B = new f32[n];
    f32* D_sim = new f32[n];
    f32* D_ref = new f32[n];

    for (int i = 0; i < n; ++i) {
        A[i] = mem.read_f32(map.A_base + i * 4);
        B[i] = mem.read_f32(map.B_base + i * 4);
        D_sim[i] = mem.read_f32(map.D_base + i * 4);
    }

    // Compute reference
    host_vsub(A, B, D_ref, n);

    // Compare
    bool valid = true;
    for (int i = 0; i < n; ++i) {
        if (!approximately_equal(D_sim[i], D_ref[i], 2)) {
            std::cerr << "Mismatch at D[" << i << "]: "
                      << "sim=" << D_sim[i] << " ref=" << D_ref[i]
                      << " (A=" << A[i] << " B=" << B[i] << ")\n";
            valid = false;
            break;
        }
    }

    delete[] A;
    delete[] B;
    delete[] D_sim;
    delete[] D_ref;

    return valid;
}

} // namespace checks
