#pragma once

#include <array>
#include <cstdint>

#include "../config.h"

namespace flip {

// Cell type used during pressure projection.
enum CellType : uint8_t {
  kSolid = 0,
  kFluid = 1,
  kAir   = 2,
};

// Marker-and-cell (staggered) grid storage.
//   u: horizontal velocity component, sampled on vertical cell edges.
//      shape (kCellsX + 1) × kCellsY.
//   v: vertical velocity component, sampled on horizontal cell edges.
//      shape kCellsX × (kCellsY + 1).
//   p: pressure at cell centres (only used during the solve).
//   s: 0.0 if cell is solid, 1.0 otherwise — used as the divergence
//      "openness" coefficient in the Gauss–Seidel sweep.
//   cell_type / particle_count: per-cell scratch for marking + rendering.
//
// All arrays are flat row-major. idx(x, y) = y * stride_x + x.
// Sized statically (stack allocation is ~5 KB at Stage 1, ~20 KB at Stage 3
// — both fit comfortably; no PSRAM gymnastics needed).
class MacGrid {
 public:
  static constexpr int kNx = kCellsX;
  static constexpr int kNy = kCellsY;
  static constexpr int kUStride = kNx + 1;          // u is (Nx+1) × Ny
  static constexpr int kVStride = kNx;              // v is Nx × (Ny+1)
  static constexpr int kUSize   = (kNx + 1) * kNy;
  static constexpr int kVSize   = kNx * (kNy + 1);
  static constexpr int kCSize   = kNx * kNy;        // cell-centred arrays

  std::array<float,   kUSize> u{};
  std::array<float,   kVSize> v{};
  std::array<float,   kUSize> u_prev{};
  std::array<float,   kVSize> v_prev{};
  std::array<float,   kUSize> u_weight{};            // P2G accumulation weights
  std::array<float,   kVSize> v_weight{};
  std::array<float,   kCSize> s{};                   // 0=solid, 1=fluid/air
  std::array<uint8_t, kCSize> cell_type{};
  // Per-cell weight from bilinear-scatter of all particles. Replaces a
  // raw integer count: each particle's float (x, y) contributes to its
  // 4 surrounding cells with bilinear weights summing to 1, so density
  // varies smoothly between cells and the renderer's LUT query produces
  // soft anti-aliased fluid edges instead of hard cell boundaries.
  std::array<float,   kCSize> density{};

  static constexpr int idxU(int x, int y) { return y * kUStride + x; }
  static constexpr int idxV(int x, int y) { return y * kVStride + x; }
  static constexpr int idxC(int x, int y) { return y * kNx + x; }
};

}  // namespace flip
