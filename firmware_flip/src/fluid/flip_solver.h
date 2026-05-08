#pragma once

#include <vector>

#include "../config.h"
#include "mac_grid.h"
#include "particle.h"

namespace flip {

// FLIP/PIC fluid solver. Algorithm follows Bridson Ch. 5 / Stam's MAC-grid
// formulation, with the FLIP velocity-update from Vateva's reference port:
//
//   particles ──advect──► P2G ──pressure-projection──► G2P ──advect──► …
//
// Each call to step() advances the simulation by exactly `dt` seconds using
// the supplied gravity vector (cells / s²). Caller is responsible for
// running step() at a fixed cadence via an accumulator (see main.cpp).
class FlipSolver {
 public:
  FlipSolver();

  // Reset state and seed `count` particles in the upper-left region of the
  // visible grid (so Stage 1 sees them fall under gravity).
  void reset(int count);

  // Advance one fixed-dt timestep with the given gravity vector.
  // Does NOT update particle_count — call recountForRender() separately
  // (the count is only needed by the renderer, and Stage 3 splits it out
  // so we can guard the publish under a mutex without holding the whole
  // physics step).
  void step(float dt, float gx, float gy);

  // Spawn one particle near the user-perceived "top" of the visible
  // grid, computed from the live gravity vector (top := opposite to
  // gravity direction). This makes the injection point follow whatever
  // way the device is rotated — landscape, portrait, upside-down — so
  // the particle always falls visually downward from the screen edge
  // the user calls "top". Caller passes the same (gx, gy) it just
  // handed to step(). Initial velocity is a small kick in gravity
  // direction. Returns false at the kMaxParticles cap.
  bool injectFromTop(float gx, float gy);

  // Repopulate grid_.density via bilinear scatter from current particle
  // positions. Each particle at float (x, y) contributes to the 4
  // enclosing cells with weights summing to 1. ~1000 particles × 4 cells
  // × a few flops = ~50 µs on ESP32-S3. Caller is expected to wrap this
  // in a mutex when physics and render run on different cores.
  void recountForRender();

  // Per-cell scattered density, indexed [y * kCellsX + x]. Render-side
  // consumers should snapshot the whole array under the same mutex used
  // around recountForRender(); see densities() below.
  float densityAt(int x, int y) const {
    return grid_.density[MacGrid::idxC(x, y)];
  }
  const std::array<float, MacGrid::kCSize>& densities() const {
    return grid_.density;
  }
  int numParticles() const { return static_cast<int>(particles_.size()); }

 private:
  void integrateParticles(float dt, float gx, float gy);
  void pushParticlesApart(int iters);
  void handleParticleCollisions();
  void transferVelocitiesToGrid();
  void solveIncompressibility(int iters, float dt, float over_relax);
  void transferVelocitiesToParticles(float flip_ratio);
  void computeDensityScatter();

  std::vector<Particle> particles_;
  MacGrid               grid_;
};

}  // namespace flip
