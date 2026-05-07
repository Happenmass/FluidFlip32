#pragma once

#include "fluid_sim_config.h"
#include <cstdint>

namespace fluidsim {

enum class CellType : uint8_t { Air, Fluid, Solid };

class FlipFluid {
 public:
  FlipFluid();
  void reset();

  // Core simulation steps (1:1 port from Rust reference).
  void integrateParticles(float dt, float xGravity, float yGravity);
  void pushParticlesApart(int numIters);
  void handleParticleCollisions();
  void transferVelocities(bool toGrid, float flipRatio);
  void updateParticleDensity();
  void solveIncompressibility(int numIters, float dt, float overRelaxation,
                              bool compensateDrift);
  void showParticles();
  void simulate(float dt, float xGravity, float yGravity, float flipRatio,
                int numPressureIters, int numParticleIters,
                float overRelaxation, bool compensateDrift);

  // Particle-array accessors used by Scene-level helpers.
  int   numParticles() const { return numParticles_; }
  void  setNumParticles(int n) { numParticles_ = n; }
  float particlePosX(int i) const { return particlePos_[2 * i]; }
  float particlePosY(int i) const { return particlePos_[2 * i + 1]; }
  void  addParticleVel(int i, float dvx, float dvy) {
    particleVel_[2 * i]     += dvx;
    particleVel_[2 * i + 1] += dvy;
  }
  void setParticlePos(int i, float x, float y) {
    particlePos_[2 * i]     = x;
    particlePos_[2 * i + 1] = y;
  }
  void setParticleVel(int i, float vx, float vy) {
    particleVel_[2 * i]     = vx;
    particleVel_[2 * i + 1] = vy;
  }
  float particleVelX(int i) const { return particleVel_[2 * i]; }
  float particleVelY(int i) const { return particleVel_[2 * i + 1]; }

  // Read-only cell type accessor for renderer feed.
  CellType cellType(int x, int y) const {
    return cellType_[x * kCellsY + y];
  }

 private:
  // Particle state.
  float particlePos_[kMaxParticles * 2];
  float particleVel_[kMaxParticles * 2];
  int   numParticles_;

  // Grid state.
  float u_[kCellsX * kCellsY];
  float v_[kCellsX * kCellsY];
  float du_[kCellsX * kCellsY];
  float dv_[kCellsX * kCellsY];
  float prevU_[kCellsX * kCellsY];
  float prevV_[kCellsX * kCellsY];
  float p_[kCellsX * kCellsY];
  float s_[kCellsX * kCellsY];
  float particleDensity_[kCellsX * kCellsY];
  CellType cellType_[kCellsX * kCellsY];
  float particleRestDensity_;

  // Spatial hash for pushParticlesApart.
  int numCellParticles_[kCellsX * kCellsY];
  int firstCellParticle_[kCellsX * kCellsY + 1];
  int cellParticleIds_[kMaxParticles];
};

class Scene {
 public:
  Scene();
  void setup(int particles);
  void simulate();  // one substep, dt = kDt

  // Public controls.
  void setGravity(float gx, float gy) { xGravity_ = gx; yGravity_ = gy; }
  void applyRadialImpulse(float cx, float cy, float strength, float radius);
  void particleAdd(int delta, int maxCap);
  int  numParticles() const { return fluid_.numParticles(); }

  // Output: per-visible-cell particle count, saturated at 255.
  // 0 = no fluid; 1+ = density tier (renderer maps to color gradient).
  using OutputGrid = uint8_t[kVisibleCellsY][kVisibleCellsX];
  void getOutput(OutputGrid& out) const;

 private:
  FlipFluid fluid_;
  float xGravity_;
  float yGravity_;
};

}  // namespace fluidsim
