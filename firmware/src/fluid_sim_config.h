#pragma once

#include <cstdint>

namespace fluidsim {

// Grid: 30×16 visible cells + 1-cell SOLID border on each side.
// 8px cells × 30 = 240 (LCD width). 8px × 16 = 128 (7px vertical letterbox).
// Larger cells per ref.jpg's chunky pixel-art aesthetic.
constexpr int kCellsX = 32;
constexpr int kCellsY = 18;

constexpr int kVisibleCellsX = 30;
constexpr int kVisibleCellsY = 16;

constexpr float kSimWidth  = static_cast<float>(kCellsX);
constexpr float kSimHeight = static_cast<float>(kCellsY);

// Pixel size of one cell on the simulation grid (collision spacing).
constexpr int kCellPixelSize = 8;

// Render size of one cell — slightly larger than the collision spacing so
// adjacent cells overlap, giving a soft blob look when alpha-blended.
// 9 px = +12.5% over the 8 px collision spacing.
constexpr int kCellRenderSize = 9;

// Fluid alpha for translucent rendering. 0 = invisible, 255 = opaque.
// 160 ≈ 63% opacity, which lets the deep-blue background bleed through and
// makes overlap regions visibly more saturated (since each pixel can be
// blended twice when neighboring cells overlap).
constexpr uint8_t kFluidAlpha = 160;

// Particles. Sized so default fill is ~30% of visible volume (ref.jpg look).
constexpr int kMaxParticles     = 600;
constexpr int kDefaultParticles = 250;

// Solver iterations and tunables (match Rust reference).
constexpr int   kPressureIters    = 10;
constexpr int   kParticleIters    = 1;
constexpr float kFlipRatio        = 0.85f;
constexpr float kOverRelaxation   = 1.9f;
constexpr float kDt               = 1.0f / 60.0f;
constexpr int   kSubstepsPerFrame = 2;

// Density.
constexpr float kDensity = 1000.0f;

// Particle radius (in cell-units). 0.5 cells = 1.0 cell diameter.
constexpr float kParticleRadius = 0.5f;

// Gravity scaling: g-units (from IMU) → sim cells / s². Tune at bringup.
constexpr float kGravityScale = 9.81f * 6.0f;

// Radial impulse (BtnA explosion). Centered on the visible grid.
// Radius covers the full screen — distance from (16,9) to any visible
// corner is ~17 cells; a slightly larger radius lets edge particles still
// feel a (linearly-falloff) kick.
constexpr float kImpulseCenterX = 16.0f;   // (kCellsX) / 2
constexpr float kImpulseCenterY = 9.0f;    // (kCellsY) / 2
constexpr float kImpulseRadius  = 20.0f;
constexpr float kImpulseStrength = 60.0f;

}  // namespace fluidsim
