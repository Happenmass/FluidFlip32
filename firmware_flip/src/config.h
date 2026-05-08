#pragma once

#include <cstdint>

// ─── Stage gating ────────────────────────────────────────────────────────────
// Stage 1: 16×8 grid, single-color render, fixed downward gravity.
// Stage 2: same grid + render, gravity from BMI270 via M5Unified.
// Stage 3 (current): 32×16 grid, 1000 particles, rim+body+sparkle render,
//                    dual-core task split.
#define FLIP_STAGE 3

// Derived feature flags.
#define FLIP_USE_IMU       (FLIP_STAGE >= 2)
#define FLIP_LARGE_GRID    (FLIP_STAGE >= 3)
#define FLIP_DENSITY_TIERS (FLIP_STAGE >= 3)

// ─── LCD geometry (M5StickS3, landscape rotation=1) ──────────────────────────
namespace flip {

constexpr int kScreenW = 240;
constexpr int kScreenH = 135;

#if FLIP_LARGE_GRID
// Stage-3 grid: 40×19 cells × (6 px wide, 7 px tall) = 240×133 — fills
// the LCD horizontally exactly (6 divides 240) with a 1 px letterbox top
// and bottom (135 − 19×7 = 2). Finer mosaic than the prior 30×15 setup
// and roughly matches the reference video's pixel density.
//
// Particle count scales with cell count to keep the bilinear-scatter
// density (avg weight ≈ 2.0/cell) the same as before, so kTargetWeight
// in the LUT formula doesn't need recalibrating. At r=0.30 natural
// close-pack is ~2.78/cell → 40×19=760 cells holds ~2110 particles max;
// 1500 → ~71% rest fill, leaving a clear waterline.
constexpr int kVisibleCellsX = 40;
constexpr int kVisibleCellsY = 19;
constexpr int kCellPxX       = 6;       // 40 × 6 = 240
constexpr int kCellPxY       = 7;       // 19 × 7 = 133  (1 px letterbox / side)
constexpr int kMaxParticles  = 1800;
constexpr int kInitParticles = 1000;
#else
// Stage-1/2 small grid, chunky pixels.
//
// kInitParticles diverges from spec's "500": at r=0.30 the natural close-
// packing density is ~2.78/cell, so 500 particles need ~180 cells but the
// visible grid only has 128 — push-apart can't satisfy min-separation, so
// every cell ends up populated. 220 → ~62% rest fill with visible
// waterline. Stage 3's bigger grid restores spec-like density.
constexpr int kVisibleCellsX = 16;
constexpr int kVisibleCellsY = 8;
constexpr int kCellPxX       = 14;     // 16×14 = 224 px wide; centered
constexpr int kCellPxY       = 14;
constexpr int kMaxParticles  = 400;
constexpr int kInitParticles = 220;
#endif

// Internal MAC grid is visible + 1-cell solid border on every side.
constexpr int kCellsX = kVisibleCellsX + 2;
constexpr int kCellsY = kVisibleCellsY + 2;

// Pixel offset to center the visible grid on the LCD. With Stage 3's
// 30×15 / 8×9 layout this is (0, 0) — exact fill. Stage 1/2's 16×8 / 14
// keeps an 8/11-ish letterbox.
constexpr int kRenderOffsetX = (kScreenW - kVisibleCellsX * kCellPxX) / 2;
constexpr int kRenderOffsetY = (kScreenH - kVisibleCellsY * kCellPxY) / 2;

// ─── Physics ─────────────────────────────────────────────────────────────────
// Cell size (h) is 1.0 in solver-internal units. Gravity below is in those
// same units per second². 9.81 m/s² scaled so the visible grid roughly maps
// to "0.5 m tall", giving recognisable falling speed at the chosen frame rate.
constexpr float kGravityMagnitude = 9.81f * 5.0f;  // cells / s²

constexpr float kFixedDt          = 1.0f / 36.0f;  // ≈ 27.6 ms (per Vateva)
constexpr float kMaxFrameDt       = 0.10f;          // accumulator clamp

constexpr float kFlipRatio        = 0.90f;
constexpr int   kPressureIters    = 20;
constexpr float kOverRelaxation   = 1.9f;

constexpr int   kPushApartIters   = 1;
constexpr float kParticleRadius   = 0.30f;          // cell-units (Vateva default)

constexpr float kRestitution      = 0.20f;          // wall bounce damping

// ─── Render: 8-level density LUT ────────────────────────────────────────────
// Per-cell colour is selected from this 8-entry table indexed by
// continuous bilinear-scatter density (NOT a binary fluid/empty). The
// gradient is intentionally REVERSE — sparse cells map to bright entries,
// dense cells map to deep entries — which is the photometry of real water
// (thin layer = bright reflection, thick volume = light absorption).
//
//   idx 0 : background deep-sea blue (no fluid)
//   idx 1 : spray / nearly-airborne droplet (close to white)
//   idx 2 : foam, very shallow water
//   idx 3 : light blue
//   idx 4 : mid blue
//   idx 5 : saturated blue
//   idx 6 : deep blue
//   idx 7 : deepest, thickest water (depth absorption)
//
// First three entries cluster in the dark→bright transition so weak
// densities don't band — RGB565 has only 5 bits of B, so spread the bright
// end and pack the dark end of the LUT.
constexpr uint16_t rgb565_pack(int r, int g, int b) {
  return static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}
constexpr uint16_t kWaterLut[8] = {
  rgb565_pack(  0,   8,  20),  // 0: bg deep sea blue (NOT pure black)
  rgb565_pack(220, 235, 255),  // 1: spray / splash near-white
  rgb565_pack(140, 190, 240),  // 2: foam / very shallow
  rgb565_pack( 80, 140, 220),  // 3: light blue
  rgb565_pack( 40,  95, 200),  // 4: mid blue
  rgb565_pack( 20,  65, 170),  // 5: saturated blue
  rgb565_pack( 10,  40, 130),  // 6: deep blue
  rgb565_pack(  5,  20,  80),  // 7: deepest depth-absorption blue
};
constexpr uint16_t kBgColor    = kWaterLut[0];     // exposed for FPS text bg
constexpr uint16_t kFluidColor = kWaterLut[3];     // legacy single-colour (Stage 1/2)

// LUT index formula (decoupled from particle count so we can tune without
// changing physics): idx = clamp((int)(density / kTargetWeight * kLutSlope), 0, 7).
// kTargetWeight is the bilinear weight a "normal" fluid cell sits at —
// for our 30×15/900-particle setup the empirical avg is ~2, so the slope
// pushes typical interior cells into the mid-LUT (idx 3-5) range and the
// dense pockets up toward 6-7. Tune by walking these numbers.
constexpr float kTargetWeight = 2.0f;       // avg interior cell weight (start here)
constexpr float kLutSlope     = 3.0f;       // density × this = LUT idx (pre-clamp)

// Surface highlight — air-water interface, brightened toward LUT[1]:
// "this cell is fluid AND at least one 4-neighbour is empty". Surface
// cells get LUT idx pulled one step toward bright (max(idx-1, 1)) and
// then a +20 RGB-channel boost on top, so the interface always reads
// brighter than the body even when the body is at LUT[3] light blue.
constexpr float kSurfaceWeight  = 1.0f;     // this cell needs at least this
constexpr float kSurfaceEmptyTh = 0.4f;     // a neighbour < this → "air"
constexpr int   kSurfaceBoost   = 20;       // RGB-channel additive boost

// Slow per-cell jitter on surface cells only — this is where the
// "波光粼粼" shimmer lives. Hash is keyed on (gx, gy, time / period) so
// the jitter pattern advances at a fixed slow rate independent of fps,
// and each cell has its own seed so adjacent cells flash out of step.
// Range ±15 starts subtle; if it looks like TV noise, raise the period.
constexpr uint32_t kJitterPeriodMs = 500;
constexpr int      kJitterRange    = 30;    // total span; result is [-15, +15)

// ─── Anti-flicker (density EMA + LUT lerp) ───────────────────────────────────
// Two-stage smoothing to kill colour flicker from particle motion.
//
//   • kDensityEmaAlpha — temporal one-pole low-pass on the bilinear
//     density field, applied each render frame:
//         ema[i] = α · ema[i] + (1 − α) · density[i]
//     0.75 settles a step in ~5 frames — invisible lag, kills the
//     "single-particle hops cell" frame-pair flicker.
//
//   • kEnableLutLerp — when true, the renderer blends the two adjacent
//     LUT entries by the fractional part of the density-derived index
//     instead of truncating. Eliminates the hard tier-edge pop when a
//     cell's density wobbles around an integer threshold.
constexpr float kDensityEmaAlpha = 0.50f;
constexpr bool  kEnableLutLerp   = true;

// ─── Particle injection (BtnA "rain") ───────────────────────────────────────
// Hold BtnA to spawn new particles at the top-centre of the visible grid.
// Caps at kMaxParticles. The injection rate is paced from the main loop
// via a millisecond timer (kInjectIntervalMs apart) feeding an atomic
// counter that the physics task drains at the head of each step.
constexpr float    kInjectVelocityY    = 8.0f;       // cells/s downward push
constexpr float    kInjectXJitter      = 1.0f;       // ±0.5 cells horizontal spread
constexpr uint32_t kInjectIntervalMs   = 100;        // 1000/10 = 10 particles/sec

// ─── Threading ───────────────────────────────────────────────────────────────
// Stage 3 spawns a physics task on core 1; the main loop on core 0 reads
// the IMU and renders. Keeping these constants here so the task config and
// the main-loop pacing stay in lockstep.
constexpr int kPhysicsTaskStack = 8192;     // bytes
constexpr int kPhysicsTaskCore  = 1;
constexpr int kPhysicsTaskPrio  = 1;

}  // namespace flip
