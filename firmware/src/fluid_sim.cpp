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
void FlipFluid::integrateParticles(float dt, float xGravity, float yGravity) {
  for (int i = 0; i < numParticles_; ++i) {
    particleVel_[2 * i]     += dt * xGravity;
    particleVel_[2 * i + 1] += dt * yGravity;
    particlePos_[2 * i]     += particleVel_[2 * i]     * dt;
    particlePos_[2 * i + 1] += particleVel_[2 * i + 1] * dt;
  }
}
void FlipFluid::pushParticlesApart(int numIters) {
  // Count particles per cell.
  std::memset(numCellParticles_, 0, sizeof(numCellParticles_));
  for (int i = 0; i < numParticles_; ++i) {
    float x = particlePos_[2 * i];
    float y = particlePos_[2 * i + 1];
    int xi = clampi(static_cast<int>(std::floor(x)), 1, kCellsX - 2);
    int yi = clampi(static_cast<int>(std::floor(y)), 1, kCellsY - 2);
    numCellParticles_[xi * kCellsY + yi]++;
  }

  // Partial sums.
  int first = 0;
  for (int i = 0; i < kCellsX * kCellsY; ++i) {
    first += numCellParticles_[i];
    firstCellParticle_[i] = first;
  }
  firstCellParticle_[kCellsX * kCellsY] = first;  // guard

  // Fill cells with particle indices.
  for (int i = 0; i < numParticles_; ++i) {
    float x = particlePos_[2 * i];
    float y = particlePos_[2 * i + 1];
    int xi = clampi(static_cast<int>(std::floor(x)), 1, kCellsX - 2);
    int yi = clampi(static_cast<int>(std::floor(y)), 1, kCellsY - 2);
    int cellNr = xi * kCellsY + yi;
    firstCellParticle_[cellNr]--;
    cellParticleIds_[firstCellParticle_[cellNr]] = i;
  }

  // Push apart.
  const float minDist  = 2.0f * kParticleRadius;
  const float minDist2 = minDist * minDist;
  for (int iter = 0; iter < numIters; ++iter) {
    for (int i = 0; i < numParticles_; ++i) {
      float px = particlePos_[2 * i];
      float py = particlePos_[2 * i + 1];
      int pxi = static_cast<int>(std::floor(px));
      int pyi = static_cast<int>(std::floor(py));
      int x0 = std::max(pxi - 1, 0);
      int y0 = std::max(pyi - 1, 0);
      int x1 = std::min(pxi + 1, kCellsX - 1);
      int y1 = std::min(pyi + 1, kCellsY - 1);
      for (int xi = x0; xi <= x1; ++xi) {
        for (int yi = y0; yi <= y1; ++yi) {
          int cellNr = xi * kCellsY + yi;
          int firstP = firstCellParticle_[cellNr];
          int lastP  = firstCellParticle_[cellNr + 1];
          for (int j = firstP; j < lastP; ++j) {
            int id = cellParticleIds_[j];
            if (id == i) continue;
            float qx = particlePos_[2 * id];
            float qy = particlePos_[2 * id + 1];
            float dx = qx - px;
            float dy = qy - py;
            float d2 = dx * dx + dy * dy;
            if (d2 > minDist2 || d2 == 0.0f) continue;
            float d = std::sqrt(d2);
            float s = 0.5f * (minDist - d) / d;
            dx *= s;
            dy *= s;
            particlePos_[2 * i]      -= dx;
            particlePos_[2 * i + 1]  -= dy;
            particlePos_[2 * id]     += dx;
            particlePos_[2 * id + 1] += dy;
          }
        }
      }
    }
  }
}
void FlipFluid::handleParticleCollisions() {}
void FlipFluid::transferVelocities(bool, float) {}
void FlipFluid::updateParticleDensity() {}
void FlipFluid::solveIncompressibility(int, float, float, bool) {}
void FlipFluid::showParticles() {}
void FlipFluid::simulate(float, float, float, float, int, int, float, bool) {}

Scene::Scene() : xGravity_(0.0f), yGravity_(0.0f) {}

void Scene::setup(int particles) {
  fluid_.reset();
  const float r  = kParticleRadius;
  const float dx = 2.0f * r;
  const float dy = 1.7320508f * r;
  int placed = 0;
  for (int j = 0; placed < particles; ++j) {
    float y = 1.0f + r + dy * j;
    if (y > kCellsY - 1.0f - r) break;
    for (int i = 0; placed < particles; ++i) {
      float x = 1.0f + r + dx * i + (j % 2 == 0 ? 0.0f : r);
      if (x > kCellsX - 1.0f - r) break;
      fluid_.setParticlePos(placed, x, y);
      fluid_.setParticleVel(placed, 0.0f, 0.0f);
      ++placed;
    }
  }
  fluid_.setNumParticles(placed);
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
