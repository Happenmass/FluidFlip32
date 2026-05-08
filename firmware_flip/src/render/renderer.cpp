#include "renderer.h"

namespace flip {

namespace {

// Unpack RGB565 → 8-bit channels.
inline void unpack565(uint16_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
  r = static_cast<uint8_t>(((c >> 11) & 0x1F) << 3);
  g = static_cast<uint8_t>(((c >>  5) & 0x3F) << 2);
  b = static_cast<uint8_t>(( c        & 0x1F) << 3);
}

inline uint16_t pack565(int r, int g, int b) {
  if (r < 0)   r = 0;   if (r > 255) r = 255;
  if (g < 0)   g = 0;   if (g > 255) g = 255;
  if (b < 0)   b = 0;   if (b > 255) b = 255;
  return static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

// Hash → signed jitter in [-kJitterRange/2, +kJitterRange/2). Uses three
// large primes for spatial mixing then a Knuth multiplicative for a final
// avalanche. The time term increments at kJitterPeriodMs intervals so the
// jitter pattern *steps* slowly rather than jittering every frame.
inline int surfaceJitter(int vx, int vy, uint32_t bucket) {
  uint32_t seed = (static_cast<uint32_t>(vx) * 73856093u)
                ^ (static_cast<uint32_t>(vy) * 19349663u)
                ^ (bucket                    * 83492791u);
  seed *= 2654435761u;
  return static_cast<int>((seed >> 24) % static_cast<uint32_t>(kJitterRange))
         - kJitterRange / 2;
}

}  // namespace

Renderer::Renderer() : sprite_(&M5.Display), ready_(false) {}

void Renderer::begin() {
  M5.Display.setRotation(1);
  sprite_.setColorDepth(16);
  sprite_.createSprite(kScreenW, kScreenH);
  ready_ = true;
}

void Renderer::draw(const float* density, int /*num_particles*/, float fps) {
  if (!ready_) return;

#if FLIP_STAGE >= 3
  // ── 0. Temporal EMA on density (suppresses flicker) ───────────────────
  // First frame: seed straight from input. After: one-pole low-pass.
  // Iterates the full MacGrid (612 cells) — the border ring stays at 0
  // because no particle ever scatters into it, so this loop is cheap.
  if (!ema_initialized_) {
    for (size_t i = 0; i < MacGrid::kCSize; ++i) ema_density_[i] = density[i];
    ema_initialized_ = true;
  } else {
    const float a   = kDensityEmaAlpha;
    const float oma = 1.f - a;
    for (size_t i = 0; i < MacGrid::kCSize; ++i) {
      ema_density_[i] = a * ema_density_[i] + oma * density[i];
    }
  }
  const float* d = ema_density_.data();

  const uint32_t bucket = millis() / kJitterPeriodMs;

  for (int vy = 0; vy < kVisibleCellsY; ++vy) {
    for (int vx = 0; vx < kVisibleCellsX; ++vx) {
      const int   cidx = MacGrid::idxC(vx + 1, vy + 1);
      const float w    = d[cidx];

      // ── 1. Floating LUT idx + surface detection (both on EMA) ───────
      float fidx = w / kTargetWeight * kLutSlope;
      if (fidx < 0.f) fidx = 0.f;
      if (fidx > 7.f) fidx = 7.f;

      bool surface = false;
      if (w >= kSurfaceWeight) {
        const int cx = vx + 1, cy = vy + 1;
        if      (cy > 1            && d[MacGrid::idxC(cx,     cy - 1)] < kSurfaceEmptyTh) surface = true;
        else if (cy + 1 < kCellsY - 1 && d[MacGrid::idxC(cx,  cy + 1)] < kSurfaceEmptyTh) surface = true;
        else if (cx > 1            && d[MacGrid::idxC(cx - 1, cy    )] < kSurfaceEmptyTh) surface = true;
        else if (cx + 1 < kCellsX - 1 && d[MacGrid::idxC(cx + 1, cy)] < kSurfaceEmptyTh) surface = true;
      }

      // ── 2. Compute base colour ─────────────────────────────────────
      // Surface cells pull idx one step toward bright; floor at 1.0 so
      // the interface never decays into LUT[0] BG.
      float base_fidx = surface ? (fidx - 1.f) : fidx;
      if (surface && base_fidx < 1.f) base_fidx = 1.f;
      if (base_fidx < 0.f) base_fidx = 0.f;
      if (base_fidx > 7.f) base_fidx = 7.f;

      int   r, g, b;
      if (kEnableLutLerp) {
        // Blend two adjacent LUT entries by the fractional part of fidx
        // — kills the hard tier-edge pop when density wobbles across an
        // integer threshold. Cost: ~25 ops/cell.
        int   i0   = static_cast<int>(base_fidx);
        if (i0 > 6) i0 = 6;                  // ensure i0+1 ≤ 7
        const float frac = base_fidx - static_cast<float>(i0);
        uint8_t r0, g0, b0, r1, g1, b1;
        unpack565(kWaterLut[i0],     r0, g0, b0);
        unpack565(kWaterLut[i0 + 1], r1, g1, b1);
        const float omf = 1.f - frac;
        r = static_cast<int>(static_cast<float>(r0) * omf + static_cast<float>(r1) * frac);
        g = static_cast<int>(static_cast<float>(g0) * omf + static_cast<float>(g1) * frac);
        b = static_cast<int>(static_cast<float>(b0) * omf + static_cast<float>(b1) * frac);
      } else {
        int idx = static_cast<int>(base_fidx);
        if (idx > 7) idx = 7;
        uint8_t rr, gg, bb;
        unpack565(kWaterLut[idx], rr, gg, bb);
        r = rr; g = gg; b = bb;
      }

      // ── 3. Surface boost + slow jitter (波光粼粼) ───────────────────
      if (surface) {
        const int j = surfaceJitter(vx, vy, bucket);
        r += kSurfaceBoost + j;
        g += kSurfaceBoost + j;
        b += kSurfaceBoost + j;
      }

      sprite_.fillRect(kRenderOffsetX + vx * kCellPxX,
                       kRenderOffsetY + vy * kCellPxY,
                       kCellPxX, kCellPxY, pack565(r, g, b));
    }
  }
#else
  // Stage-1/2 fallback: flat single-colour fill on top of bg.
  sprite_.fillScreen(kBgColor);
  for (int vy = 0; vy < kVisibleCellsY; ++vy) {
    for (int vx = 0; vx < kVisibleCellsX; ++vx) {
      const float w = density[MacGrid::idxC(vx + 1, vy + 1)];
      if (w < 0.5f) continue;
      sprite_.fillRect(kRenderOffsetX + vx * kCellPxX,
                       kRenderOffsetY + vy * kCellPxY,
                       kCellPxX, kCellPxY, kFluidColor);
    }
  }
#endif

  // FPS readout (top-left, small). Drawn last so it sits above fluid.
  sprite_.setTextColor(0xFFFF, kBgColor);
  sprite_.setTextSize(1);
  sprite_.setCursor(2, 2);
  sprite_.printf("%2.0f", fps);

  sprite_.pushSprite(0, 0);
}

}  // namespace flip
