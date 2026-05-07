#include "fluid_sim.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fluidsim {

namespace {
inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline int clampi(int x, int lo, int hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
}  // namespace

FlipFluid::FlipFluid() { reset(); }

void FlipFluid::reset() {
  std::memset(particlePos_, 0, sizeof(particlePos_));
  std::memset(particleVel_, 0, sizeof(particleVel_));
  std::memset(u_,  0, sizeof(u_));
  std::memset(v_,  0, sizeof(v_));
  std::memset(du_, 0, sizeof(du_));
  std::memset(dv_, 0, sizeof(dv_));
  std::memset(prevU_, 0, sizeof(prevU_));
  std::memset(prevV_, 0, sizeof(prevV_));
  std::memset(p_, 0, sizeof(p_));
  std::memset(particleDensity_, 0, sizeof(particleDensity_));
  std::memset(numCellParticles_, 0, sizeof(numCellParticles_));
  std::memset(firstCellParticle_, 0, sizeof(firstCellParticle_));
  std::memset(cellParticleIds_, 0, sizeof(cellParticleIds_));

  // s_ marks fluid (1.0) vs solid (0.0). Border = solid.
  for (int x = 0; x < kCellsX; ++x) {
    for (int y = 0; y < kCellsY; ++y) {
      bool border = (x == 0 || x == kCellsX - 1 || y == 0 || y == kCellsY - 1);
      s_[x * kCellsY + y] = border ? 0.0f : 1.0f;
      cellType_[x * kCellsY + y] = border ? CellType::Solid : CellType::Air;
    }
  }
  particleRestDensity_ = 0.0f;
  numParticles_ = 0;
}

// Stubs — implemented in subsequent tasks.
void FlipFluid::integrateParticles(float, float, float) {}
void FlipFluid::pushParticlesApart(int) {}
void FlipFluid::handleParticleCollisions() {}
void FlipFluid::transferVelocities(bool, float) {}
void FlipFluid::updateParticleDensity() {}
void FlipFluid::solveIncompressibility(int, float, float, bool) {}
void FlipFluid::showParticles() {}
void FlipFluid::simulate(float, float, float, float, int, int, float, bool) {}

Scene::Scene() : xGravity_(0.0f), yGravity_(0.0f) {}

void Scene::setup(int particles) {
  fluid_.reset();
  // Seed particles in a hex-packed block in the upper-left of the visible area.
  // Real implementation lands in Task 3 — for now, just record requested count.
  fluid_.setNumParticles(particles);
  xGravity_ = 0.0f;
  yGravity_ = 0.0f;
}

void Scene::simulate() {
  fluid_.simulate(kDt, xGravity_, yGravity_, kFlipRatio,
                  kPressureIters, kParticleIters, kOverRelaxation, true);
}

void Scene::applyRadialImpulse(float, float, float, float) {}
void Scene::particleAdd(int, int) {}

void Scene::getOutput(OutputGrid& out) const {
  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      // Visible cells start at index 1 in both dimensions.
      out[y][x] = fluid_.cellType(x + 1, y + 1) == CellType::Fluid;
    }
  }
}

}  // namespace fluidsim
