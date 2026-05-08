#pragma once

#include <cstdint>

namespace fluidsim {

// Grid: 40×22 visible cells + 1-cell SOLID border on each side.
// 6px cells × 40 = 240 (LCD width). 6px × 22 = 132 (3px vertical letterbox).
// Smaller cells than the chunky-pixel ref for finer surface detail.
constexpr int kCellsX = 42;
constexpr int kCellsY = 24;

constexpr int kVisibleCellsX = 40;
constexpr int kVisibleCellsY = 22;

constexpr float kSimWidth  = static_cast<float>(kCellsX);
constexpr float kSimHeight = static_cast<float>(kCellsY);

// Pixel size of one cell on the simulation grid (collision spacing).
constexpr int kCellPixelSize = 6;

// Render size: each cell drawn slightly larger than the collision cell so
// adjacent cells overlap by 1 px on every side. Combined with depth-ordered
// alpha layering this softens the surface outline and yields a blob look.
// Must be >= kCellPixelSize and have the same parity for symmetric expand.
constexpr int kCellRenderSize = 8;

// Master toggle for translucent / blob rendering. false = opaque rim+body
// + sparkle (matches reference photo aesthetic, cheap); true = depth-tier
// alpha blending + density jitter (soft halo, ~2–3× slower on the render
// path). Compiled away — flip and rebuild.
constexpr bool kEnableAlphaBlending = true;

// --- Opaque (rim+body+sparkle) style tunables. Only used when
// kEnableAlphaBlending == false. RGB565 = (R5<<11) | (G6<<5) | B5.
constexpr uint16_t kOpaqueBgColor    = 0x0010;  // bg deep navy        (0,  0, 16)
constexpr uint16_t kOpaqueBodyColor  = 0x355F;  // body mid blue-cyan  (6, 42, 31)
constexpr uint16_t kOpaqueRimColor   = 0xFFFF;  // air-touching rim    (white)
constexpr uint16_t kOpaqueSparkColor = 0xFFFF;  // sparkle dot         (white)

// Sparkle chance per body cell per frame, in percent (0–100).
constexpr int kOpaqueSparkleChancePct = 15;
// Sparkle dot size in pixels (drawn as fillRect inside the body cell).
constexpr int kOpaqueSparkleSize      = 2;
// Sparkle frame period in ms — sparkle pattern advances on this clock,
// independent of the actual render fps so flicker rate stays steady.
constexpr uint32_t kOpaqueSparkleFrameMs = 60;

// Particles. Visible-area scaled from prior 30×16/250 default (~45% lattice
// fill in a hexagonal pack); 40×22 grid keeps the same fill ratio.
constexpr int kMaxParticles     = 1000;
constexpr int kDefaultParticles = 450;

// Solver iterations and tunables (match Rust reference).
constexpr int   kPressureIters    = 6;
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

// BtnA "anti-gravity launch": every particle's velocity is set to a vector
// pointing opposite to current gravity, with this magnitude (sim cells / s).
// 40 ≈ the speed reached after ~0.7 s of free fall under kGravityScale, so
// the kick visibly reverses recent falling motion without flinging
// particles into the walls.
constexpr float kAntiGravityVelocity = 40.0f;

}  // namespace fluidsim
