#pragma once

#include <cstdint>

namespace fluidsim {

// Grid: 40×22 visible cells + 1-cell SOLID border on each side.
constexpr int kCellsX = 42;
constexpr int kCellsY = 24;

constexpr int kVisibleCellsX = 40;
constexpr int kVisibleCellsY = 22;

constexpr float kSimWidth  = static_cast<float>(kCellsX);
constexpr float kSimHeight = static_cast<float>(kCellsY);

// Pixel size of one cell when rendered.
constexpr int kCellPixelSize = 6;

// Particles.
constexpr int kMaxParticles     = 1200;
constexpr int kDefaultParticles = 800;

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

// Radial impulse (BtnA explosion).
constexpr float kImpulseCenterX = 21.0f;
constexpr float kImpulseCenterY = 12.0f;
constexpr float kImpulseRadius  = 12.0f;
constexpr float kImpulseStrength = 60.0f;  // tune at bringup

// Shake detection.
constexpr float kShakeMagnitudeG  = 1.8f;
constexpr int   kShakeFramesNeeded = 8;
constexpr int   kShakeAddParticles = 400;

}  // namespace fluidsim
