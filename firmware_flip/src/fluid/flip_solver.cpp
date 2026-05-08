#include "flip_solver.h"

#include <cmath>

namespace flip {

namespace {

// Bilinear weights for a sample point (fx, fy) ∈ [0, 1)².
struct BilinearW { float w00, w10, w01, w11; };
inline BilinearW bilin(float fx, float fy) {
  return { (1.f - fx) * (1.f - fy), fx * (1.f - fy),
           (1.f - fx) *        fy , fx *        fy  };
}

// Convert a particle position (px, py) in cell-units to bilinear sampling
// inputs for a staggered field offset by (offX, offY) cell-units. Returns
// integer base indices (i, j) plus fractional weights.
struct StaggerSample { int i, j; BilinearW w; };
inline StaggerSample stagger(float px, float py, float offX, float offY,
                             int max_i, int max_j) {
  float gx = px - offX;
  float gy = py - offY;
  int i = static_cast<int>(std::floor(gx));
  int j = static_cast<int>(std::floor(gy));
  // Clamp so (i+1, j+1) stays in-range; particles get nudged off the very
  // edge by collision handling so this almost never trips, but it keeps the
  // sample safe in pathological cases (e.g. NaN propagation guards).
  if (i < 0) i = 0;
  if (j < 0) j = 0;
  if (i > max_i - 1) i = max_i - 1;
  if (j > max_j - 1) j = max_j - 1;
  float fx = gx - static_cast<float>(i);
  float fy = gy - static_cast<float>(j);
  if (fx < 0.f) fx = 0.f;
  if (fy < 0.f) fy = 0.f;
  if (fx > 1.f) fx = 1.f;
  if (fy > 1.f) fy = 1.f;
  return { i, j, bilin(fx, fy) };
}

}  // namespace

FlipSolver::FlipSolver() { reset(kInitParticles); }

void FlipSolver::reset(int count) {
  particles_.clear();
  particles_.reserve(kMaxParticles);

  // Seed particles row-by-row from the top of the visible grid until we hit
  // the requested count. Spacing 0.45 cells on a square lattice → ~5
  // particles per cell, which is denser than the natural rest-state at
  // r=0.30 (push-apart relaxes the slight overlap during the first frame).
  // Filling the upper ~70% of the grid lets Stage-1 verification observe
  // particles falling under gravity and piling at the bottom.
  const float spacing = 0.45f;
  const float xLo     = 1.0f + spacing * 0.5f;
  const float xHi     = static_cast<float>(kCellsX - 1) - spacing * 0.5f;
  const float yLo     = 1.0f + spacing * 0.5f;
  const float yHi     = 1.0f + static_cast<float>(kVisibleCellsY) * 0.70f;

  for (float py = yLo; py <= yHi; py += spacing) {
    for (float px = xLo; px <= xHi; px += spacing) {
      if (static_cast<int>(particles_.size()) >= count) goto seeded;
      particles_.push_back({ px, py, 0.f, 0.f });
    }
  }
seeded:;

  // Init MAC grid: solid border ring, fluid/air interior. s=0 for solid,
  // s=1 elsewhere — the projection step uses s as the divergence "openness"
  // coefficient. cell_type stays in sync (kSolid on the border).
  grid_.s.fill(1.f);
  grid_.cell_type.fill(kAir);
  for (int x = 0; x < kCellsX; ++x) {
    grid_.s[MacGrid::idxC(x, 0)]            = 0.f;
    grid_.s[MacGrid::idxC(x, kCellsY - 1)]  = 0.f;
    grid_.cell_type[MacGrid::idxC(x, 0)]            = kSolid;
    grid_.cell_type[MacGrid::idxC(x, kCellsY - 1)]  = kSolid;
  }
  for (int y = 0; y < kCellsY; ++y) {
    grid_.s[MacGrid::idxC(0, y)]            = 0.f;
    grid_.s[MacGrid::idxC(kCellsX - 1, y)]  = 0.f;
    grid_.cell_type[MacGrid::idxC(0, y)]            = kSolid;
    grid_.cell_type[MacGrid::idxC(kCellsX - 1, y)]  = kSolid;
  }

  grid_.u.fill(0.f);
  grid_.v.fill(0.f);
  grid_.u_prev.fill(0.f);
  grid_.v_prev.fill(0.f);
  grid_.density.fill(0.f);
}

void FlipSolver::step(float dt, float gx, float gy) {
  integrateParticles(dt, gx, gy);
  pushParticlesApart(kPushApartIters);
  handleParticleCollisions();
  transferVelocitiesToGrid();
  solveIncompressibility(kPressureIters, dt, kOverRelaxation);
  transferVelocitiesToParticles(kFlipRatio);
}

void FlipSolver::recountForRender() { computeDensityScatter(); }

// ─── Particle injection (BtnA-driven, orientation-aware) ────────────────────
// Drops a fresh particle near the user-perceived top of the screen — i.e.,
// opposite to the current gravity vector — with a kick along gravity and
// a small jitter perpendicular to gravity. Universal across device
// rotations: in landscape upright gravity is (0, +g) so spawn lands near
// firmware (x=center, y=small); in portrait it lands near (x=small,
// y=center); etc. Caller is responsible for not calling once the max
// cap is hit, but the check is here for safety.
bool FlipSolver::injectFromTop(float gx, float gy) {
  if (static_cast<int>(particles_.size()) >= kMaxParticles) return false;

  // ── 1. Up direction = -normalize(gravity) ──────────────────────────────
  // If gravity is essentially zero (device flat on the table), fall back
  // to firmware -y as "up" so the injection still has a sensible default
  // and matches the most common landscape grip.
  const float gmag = std::sqrt(gx * gx + gy * gy);
  float ux, uy;
  if (gmag > 1.f) {
    ux = -gx / gmag;
    uy = -gy / gmag;
  } else {
    ux = 0.f;
    uy = -1.f;
  }

  // ── 2. Spawn position = grid centre + reach × up ──────────────────────
  // reach is bounded by the smaller half-dimension of the grid so the
  // spawn point stays inside the rectangle no matter which axis is "up".
  const float center_x = static_cast<float>(kCellsX) * 0.5f;
  const float center_y = static_cast<float>(kCellsY) * 0.5f;
  const float reach    = std::min(center_x, center_y) - 1.5f;

  // ── 3. Lateral jitter along the perpendicular of up ───────────────────
  // perp = up rotated 90° (perp_x, perp_y) = (-uy, ux). Consecutive
  // injections fan out along the top edge instead of stacking at one
  // pixel, which would otherwise produce a vertical line of particles.
  static uint32_t lcg = 0x12345678u;
  lcg = lcg * 1103515245u + 12345u;
  const float r01    = static_cast<float>((lcg >> 16) & 0xFFFF) / 65535.f;
  const float jitter = (r01 - 0.5f) * kInjectXJitter;
  const float perp_x = -uy;
  const float perp_y =  ux;

  Particle p;
  p.x = center_x + ux * reach + perp_x * jitter;
  p.y = center_y + uy * reach + perp_y * jitter;

  // ── 4. Initial velocity along gravity (kick) ──────────────────────────
  if (gmag > 1.f) {
    p.vx = (gx / gmag) * kInjectVelocityY;
    p.vy = (gy / gmag) * kInjectVelocityY;
  } else {
    p.vx = 0.f;
    p.vy = kInjectVelocityY;
  }

  particles_.push_back(p);
  return true;
}

// ─── Step 1: gravity + ballistic advection ──────────────────────────────────
void FlipSolver::integrateParticles(float dt, float gx, float gy) {
  for (auto& p : particles_) {
    p.vx += gx * dt;
    p.vy += gy * dt;
    p.x  += p.vx * dt;
    p.y  += p.vy * dt;
  }
}

// ─── Step 2: particle separation via spatial hash ────────────────────────────
// Cell size for the hash equals 2× particle radius. Every particle pair
// closer than 2r is pushed apart by half the overlap each. Repeated `iters`
// times to converge. Skipped at iters == 0.
void FlipSolver::pushParticlesApart(int iters) {
  if (iters <= 0 || particles_.empty()) return;

  const float r        = kParticleRadius;
  const float min_dist = 2.f * r;
  const float min_d2   = min_dist * min_dist;

  // Hash cells span the full sim grid. Cell size = 2r → grid resolution
  // (kCellsX / 2r) × (kCellsY / 2r). Use a per-bucket head + linked-list
  // (next-pointer style) so memory is O(N + buckets).
  const float inv_h     = 1.f / min_dist;
  const int   hx        = static_cast<int>(std::ceil(kCellsX * inv_h)) + 1;
  const int   hy        = static_cast<int>(std::ceil(kCellsY * inv_h)) + 1;
  const int   bucket_n  = hx * hy;

  static thread_local std::vector<int> head;
  static thread_local std::vector<int> next_idx;
  head.assign(bucket_n, -1);
  next_idx.assign(particles_.size(), -1);

  auto bucket = [&](float x, float y) {
    int bx = static_cast<int>(x * inv_h);
    int by = static_cast<int>(y * inv_h);
    if (bx < 0) bx = 0; if (bx >= hx) bx = hx - 1;
    if (by < 0) by = 0; if (by >= hy) by = hy - 1;
    return by * hx + bx;
  };

  // Build hash.
  for (int i = 0; i < static_cast<int>(particles_.size()); ++i) {
    const int b = bucket(particles_[i].x, particles_[i].y);
    next_idx[i] = head[b];
    head[b]     = i;
  }

  for (int it = 0; it < iters; ++it) {
    for (int i = 0; i < static_cast<int>(particles_.size()); ++i) {
      Particle& a = particles_[i];
      int bx = static_cast<int>(a.x * inv_h);
      int by = static_cast<int>(a.y * inv_h);
      for (int oy = -1; oy <= 1; ++oy) {
        for (int ox = -1; ox <= 1; ++ox) {
          int nx = bx + ox, ny = by + oy;
          if (nx < 0 || ny < 0 || nx >= hx || ny >= hy) continue;
          for (int j = head[ny * hx + nx]; j != -1; j = next_idx[j]) {
            if (j <= i) continue;  // each pair once
            Particle& b = particles_[j];
            const float dx = b.x - a.x;
            const float dy = b.y - a.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 >= min_d2 || d2 == 0.f) continue;
            const float d    = std::sqrt(d2);
            const float push = 0.5f * (min_dist - d) / d;
            a.x -= dx * push;
            a.y -= dy * push;
            b.x += dx * push;
            b.y += dy * push;
          }
        }
      }
    }
  }
}

// ─── Step 3: clamp to interior + reflect off walls ──────────────────────────
void FlipSolver::handleParticleCollisions() {
  const float r    = kParticleRadius;
  const float xLo  = 1.f + r;
  const float xHi  = static_cast<float>(kCellsX - 1) - r;
  const float yLo  = 1.f + r;
  const float yHi  = static_cast<float>(kCellsY - 1) - r;
  const float rest = kRestitution;
  for (auto& p : particles_) {
    if (p.x < xLo) { p.x = xLo; if (p.vx < 0) p.vx = -p.vx * rest; }
    if (p.x > xHi) { p.x = xHi; if (p.vx > 0) p.vx = -p.vx * rest; }
    if (p.y < yLo) { p.y = yLo; if (p.vy < 0) p.vy = -p.vy * rest; }
    if (p.y > yHi) { p.y = yHi; if (p.vy > 0) p.vy = -p.vy * rest; }
  }
}

// ─── Step 4: P2G — scatter particle velocities onto MAC edges ───────────────
// u-grid samples at (i, j+0.5) — offset (0, 0.5) in cell-units.
// v-grid samples at (i+0.5, j) — offset (0.5, 0).
void FlipSolver::transferVelocitiesToGrid() {
  grid_.u.fill(0.f);
  grid_.v.fill(0.f);
  grid_.u_weight.fill(0.f);
  grid_.v_weight.fill(0.f);

  // Mark cells: any cell with at least one particle becomes fluid; solid
  // border stays solid; everything else is air. Keep the prior particle
  // count buffer alive for renderer consumers between steps.
  for (int idx = 0; idx < MacGrid::kCSize; ++idx) {
    if (grid_.cell_type[idx] != kSolid) grid_.cell_type[idx] = kAir;
  }

  for (const auto& p : particles_) {
    // Mark containing cell as fluid.
    int cx = static_cast<int>(p.x);
    int cy = static_cast<int>(p.y);
    if (cx >= 0 && cx < kCellsX && cy >= 0 && cy < kCellsY) {
      uint8_t& t = grid_.cell_type[MacGrid::idxC(cx, cy)];
      if (t == kAir) t = kFluid;
    }

    // Scatter u (offset 0, 0.5 — clamp i to [0, kCellsX-1] so i+1 ≤ kCellsX).
    {
      auto s = stagger(p.x, p.y, 0.f, 0.5f, kCellsX, kCellsY - 1);
      const float vx = p.vx;
      grid_.u       [MacGrid::idxU(s.i,     s.j    )] += vx * s.w.w00;
      grid_.u       [MacGrid::idxU(s.i + 1, s.j    )] += vx * s.w.w10;
      grid_.u       [MacGrid::idxU(s.i,     s.j + 1)] += vx * s.w.w01;
      grid_.u       [MacGrid::idxU(s.i + 1, s.j + 1)] += vx * s.w.w11;
      grid_.u_weight[MacGrid::idxU(s.i,     s.j    )] += s.w.w00;
      grid_.u_weight[MacGrid::idxU(s.i + 1, s.j    )] += s.w.w10;
      grid_.u_weight[MacGrid::idxU(s.i,     s.j + 1)] += s.w.w01;
      grid_.u_weight[MacGrid::idxU(s.i + 1, s.j + 1)] += s.w.w11;
    }
    // Scatter v (offset 0.5, 0 — clamp j to [0, kCellsY-1] so j+1 ≤ kCellsY).
    {
      auto s = stagger(p.x, p.y, 0.5f, 0.f, kCellsX - 1, kCellsY);
      const float vy = p.vy;
      grid_.v       [MacGrid::idxV(s.i,     s.j    )] += vy * s.w.w00;
      grid_.v       [MacGrid::idxV(s.i + 1, s.j    )] += vy * s.w.w10;
      grid_.v       [MacGrid::idxV(s.i,     s.j + 1)] += vy * s.w.w01;
      grid_.v       [MacGrid::idxV(s.i + 1, s.j + 1)] += vy * s.w.w11;
      grid_.v_weight[MacGrid::idxV(s.i,     s.j    )] += s.w.w00;
      grid_.v_weight[MacGrid::idxV(s.i + 1, s.j    )] += s.w.w10;
      grid_.v_weight[MacGrid::idxV(s.i,     s.j + 1)] += s.w.w01;
      grid_.v_weight[MacGrid::idxV(s.i + 1, s.j + 1)] += s.w.w11;
    }
  }

  // Normalize: u[i] /= sum_w[i] (or 0 if no contribution).
  for (int i = 0; i < MacGrid::kUSize; ++i) {
    grid_.u[i] = grid_.u_weight[i] > 0.f ? grid_.u[i] / grid_.u_weight[i] : 0.f;
  }
  for (int i = 0; i < MacGrid::kVSize; ++i) {
    grid_.v[i] = grid_.v_weight[i] > 0.f ? grid_.v[i] / grid_.v_weight[i] : 0.f;
  }

  // Save pre-projection grid for FLIP delta.
  grid_.u_prev = grid_.u;
  grid_.v_prev = grid_.v;
}

// ─── Step 5: pressure projection (Gauss–Seidel on divergence) ───────────────
// Standard incompressible-fluid relaxation. The s coefficient (0=solid,
// 1=fluid/air) zeros contributions from solid neighbours so velocity isn't
// pushed through walls.
void FlipSolver::solveIncompressibility(int iters, float /*dt*/,
                                        float over_relax) {
  for (int it = 0; it < iters; ++it) {
    for (int y = 1; y < kCellsY - 1; ++y) {
      for (int x = 1; x < kCellsX - 1; ++x) {
        if (grid_.cell_type[MacGrid::idxC(x, y)] != kFluid) continue;

        const float sx0 = grid_.s[MacGrid::idxC(x - 1, y)];
        const float sx1 = grid_.s[MacGrid::idxC(x + 1, y)];
        const float sy0 = grid_.s[MacGrid::idxC(x, y - 1)];
        const float sy1 = grid_.s[MacGrid::idxC(x, y + 1)];
        const float s_total = sx0 + sx1 + sy0 + sy1;
        if (s_total == 0.f) continue;

        const float div =
            grid_.u[MacGrid::idxU(x + 1, y)] - grid_.u[MacGrid::idxU(x, y)] +
            grid_.v[MacGrid::idxV(x, y + 1)] - grid_.v[MacGrid::idxV(x, y)];
        const float p = -div / s_total * over_relax;

        grid_.u[MacGrid::idxU(x,     y)]     -= sx0 * p;
        grid_.u[MacGrid::idxU(x + 1, y)]     += sx1 * p;
        grid_.v[MacGrid::idxV(x,     y)]     -= sy0 * p;
        grid_.v[MacGrid::idxV(x,     y + 1)] += sy1 * p;
      }
    }
  }
}

// ─── Step 6: G2P — gather projected grid velocities back onto particles ─────
// new_v = (1 - flip) * PIC + flip * (old_v + Δgrid).
void FlipSolver::transferVelocitiesToParticles(float flip_ratio) {
  const float pic_ratio = 1.f - flip_ratio;
  for (auto& p : particles_) {
    // u-component sample (offset 0, 0.5).
    {
      auto s = stagger(p.x, p.y, 0.f, 0.5f, kCellsX, kCellsY - 1);
      const float u00 = grid_.u[MacGrid::idxU(s.i,     s.j    )];
      const float u10 = grid_.u[MacGrid::idxU(s.i + 1, s.j    )];
      const float u01 = grid_.u[MacGrid::idxU(s.i,     s.j + 1)];
      const float u11 = grid_.u[MacGrid::idxU(s.i + 1, s.j + 1)];
      const float p00 = grid_.u_prev[MacGrid::idxU(s.i,     s.j    )];
      const float p10 = grid_.u_prev[MacGrid::idxU(s.i + 1, s.j    )];
      const float p01 = grid_.u_prev[MacGrid::idxU(s.i,     s.j + 1)];
      const float p11 = grid_.u_prev[MacGrid::idxU(s.i + 1, s.j + 1)];
      const float pic = u00 * s.w.w00 + u10 * s.w.w10 +
                        u01 * s.w.w01 + u11 * s.w.w11;
      const float du  = (u00 - p00) * s.w.w00 + (u10 - p10) * s.w.w10 +
                        (u01 - p01) * s.w.w01 + (u11 - p11) * s.w.w11;
      p.vx = pic_ratio * pic + flip_ratio * (p.vx + du);
    }
    // v-component sample (offset 0.5, 0).
    {
      auto s = stagger(p.x, p.y, 0.5f, 0.f, kCellsX - 1, kCellsY);
      const float v00 = grid_.v[MacGrid::idxV(s.i,     s.j    )];
      const float v10 = grid_.v[MacGrid::idxV(s.i + 1, s.j    )];
      const float v01 = grid_.v[MacGrid::idxV(s.i,     s.j + 1)];
      const float v11 = grid_.v[MacGrid::idxV(s.i + 1, s.j + 1)];
      const float p00 = grid_.v_prev[MacGrid::idxV(s.i,     s.j    )];
      const float p10 = grid_.v_prev[MacGrid::idxV(s.i + 1, s.j    )];
      const float p01 = grid_.v_prev[MacGrid::idxV(s.i,     s.j + 1)];
      const float p11 = grid_.v_prev[MacGrid::idxV(s.i + 1, s.j + 1)];
      const float pic = v00 * s.w.w00 + v10 * s.w.w10 +
                        v01 * s.w.w01 + v11 * s.w.w11;
      const float dv  = (v00 - p00) * s.w.w00 + (v10 - p10) * s.w.w10 +
                        (v01 - p01) * s.w.w01 + (v11 - p11) * s.w.w11;
      p.vy = pic_ratio * pic + flip_ratio * (p.vy + dv);
    }
  }
}

// ─── Step 7: per-cell density via bilinear scatter (renderer-only) ──────────
// Each particle's float position (x, y) contributes to its 4 surrounding
// cells with bilinear weights summing to 1, so density varies smoothly
// between cells. That smoothness is what lets the renderer's 8-level LUT
// produce soft anti-aliased fluid edges and the "波光粼粼" reading.
//
// Wall-mirror compensation: the bilinear kernel for a particle near a
// wall would, in an unbounded grid, deposit some weight on the OOB side.
// In our bounded grid that weight is dropped → wall-adjacent visible
// cells systematically read ~50% of the density of equivalent interior
// cells (corners read ~25%). Each near-wall particle therefore also
// scatters one or more mirrored copies of itself reflected across the
// wall(s) it's close to — that puts the missing weight back into the
// wall-adjacent / corner cells. Math: 4 single-axis mirrors handle each
// edge cell, 4 corner double-mirrors handle the 4 corner cells.
void FlipSolver::computeDensityScatter() {
  grid_.density.fill(0.f);

  // Inside-facing edges of the 1-cell-thick solid border.
  constexpr float xL = 1.0f;
  const     float xR = static_cast<float>(kCellsX - 1);
  constexpr float yT = 1.0f;
  const     float yB = static_cast<float>(kCellsY - 1);

  // Bilinear-deposit a single (real or virtual) particle into 4 cells.
  // Out-of-range writes silently dropped so corner mirrors that fall off
  // the far side of the grid are safe.
  auto scatter = [this](float px, float py) {
    const int sgx = static_cast<int>(std::floor(px));
    const int sgy = static_cast<int>(std::floor(py));
    if (sgx < 0 || sgy < 0 || sgx + 1 >= kCellsX || sgy + 1 >= kCellsY) return;
    const float sfx = px - static_cast<float>(sgx);
    const float sfy = py - static_cast<float>(sgy);
    grid_.density[MacGrid::idxC(sgx,     sgy    )] += (1.f - sfx) * (1.f - sfy);
    grid_.density[MacGrid::idxC(sgx + 1, sgy    )] += sfx         * (1.f - sfy);
    grid_.density[MacGrid::idxC(sgx,     sgy + 1)] += (1.f - sfx) * sfy;
    grid_.density[MacGrid::idxC(sgx + 1, sgy + 1)] += sfx         * sfy;
  };

  for (const auto& p : particles_) {
    // 1) Real particle.
    scatter(p.x, p.y);

    // 2) Single-axis wall mirrors. A particle within 1 cell of a wall
    // gets a mirror image across that wall; bilinearly scattering it
    // restores the weight the wall would otherwise drop.
    const bool nL = p.x <  xL + 1.0f;     // near left wall
    const bool nR = p.x >  xR - 1.0f;     // near right wall
    const bool nT = p.y <  yT + 1.0f;     // near top wall
    const bool nB = p.y >  yB - 1.0f;     // near bottom wall

    if (nL) scatter(2.0f * xL - p.x, p.y);
    if (nR) scatter(2.0f * xR - p.x, p.y);
    if (nT) scatter(p.x, 2.0f * yT - p.y);
    if (nB) scatter(p.x, 2.0f * yB - p.y);

    // 3) Corner double-mirrors — only fire when a particle is in a 1×1
    // corner cell. Each corner cell needs 4× compensation total: real
    // (×1) + two single-axis mirrors (×2) + this diagonal mirror (×1).
    if (nL && nT) scatter(2.0f * xL - p.x, 2.0f * yT - p.y);
    if (nR && nT) scatter(2.0f * xR - p.x, 2.0f * yT - p.y);
    if (nL && nB) scatter(2.0f * xL - p.x, 2.0f * yB - p.y);
    if (nR && nB) scatter(2.0f * xR - p.x, 2.0f * yB - p.y);
  }
}

}  // namespace flip
