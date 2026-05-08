#include "renderer.h"

namespace fluidsim {

// Two render styles, selected by kEnableAlphaBlending:
//  • false (default): opaque rim+body+sparkle. Cells touching air glow
//    bright white; interior is a flat mid-cyan body; random hash-driven
//    sparkle dots flicker on top of the body for the "particle motion"
//    feel seen in the reference photo.
//  • true: depth-tier alpha layering. Chamfer Manhattan distance from
//    air drives a multi-tier color+alpha gradient; cell render rect is
//    larger than the collision rect so adjacent cells overlap and
//    accumulate into a soft blob halo.
//
// Channel layout: RGB565 = (R5 << 11) | (G6 << 5) | B5.

constexpr uint16_t kBgColor = kOpaqueBgColor;

// Alpha-mode tiers (only consulted when kEnableAlphaBlending == true).
// 0 = empty (skipped); 1 = touches air (shallow); 5+ = deep core.
constexpr int kDepthTiers = 6;
constexpr uint16_t kDepthColors[kDepthTiers] = {
  0x0010,  // 0: bg (unused, fluid cells never use this)
  0x57FF,  // 1: pale haze at surface       (R=10, G=63, B=31)
  0x37FF,  // 2: less pale, more cyan       (R= 6, G=63, B=31)
  0x07FF,  // 3: pure cyan                  (R= 0, G=63, B=31)
  0x17FF,  // 4: pure cyan + faint glow     (R= 2, G=63, B=31)
  0x47FF,  // 5+: white-cyan core glow      (R= 8, G=63, B=31)
};
constexpr uint8_t kDepthAlpha[kDepthTiers] = {
  0,    // 0: skipped
  84,   // 1: ~25% — translucent surface haze
  112,  // 2: ~44%
  160,  // 3: ~63%
  208,  // 4: ~82%
  240,  // 5+: ~94% — saturated interior core
};

Renderer::Renderer() : sprite_(&M5.Display), initialized_(false) {}

void Renderer::begin() {
  M5.Display.setRotation(1);                      // 240×135 landscape
  sprite_.setColorDepth(16);
  sprite_.createSprite(M5.Display.width(), M5.Display.height());
  initialized_ = true;
}

void Renderer::draw(const Scene::OutputGrid& grid) {
  if (!initialized_) return;
  sprite_.fillScreen(kBgColor);

  // Step 1: occupancy mask + morphological closing (dilate→erode) to
  // suppress 1-cell pinholes from FLIP particle-count jitter so interior
  // cells don't strobe between body and bg between frames.
  uint8_t mask[kVisibleCellsY][kVisibleCellsX];
  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      mask[y][x] = grid[y][x] > 0 ? 1 : 0;
    }
  }
  // Dilate (4-conn, OOB=0).
  uint8_t dilated[kVisibleCellsY][kVisibleCellsX];
  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      dilated[y][x] = mask[y][x]
        || (y > 0                  && mask[y - 1][x])
        || (y < kVisibleCellsY - 1 && mask[y + 1][x])
        || (x > 0                  && mask[y][x - 1])
        || (x < kVisibleCellsX - 1 && mask[y][x + 1]);
    }
  }
  // Erode (4-conn, OOB=1) to preserve real wall-adjacent surface.
  uint8_t fluid[kVisibleCellsY][kVisibleCellsX];
  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      const uint8_t up = (y > 0)                  ? dilated[y - 1][x] : 1;
      const uint8_t dn = (y < kVisibleCellsY - 1) ? dilated[y + 1][x] : 1;
      const uint8_t lt = (x > 0)                  ? dilated[y][x - 1] : 1;
      const uint8_t rt = (x < kVisibleCellsX - 1) ? dilated[y][x + 1] : 1;
      fluid[y][x] = dilated[y][x] && up && dn && lt && rt;
    }
  }

  const int xOff   = 0;
  const int yOff   = (M5.Display.height() - kVisibleCellsY * kCellPixelSize) / 2;
  const int expand = (kCellRenderSize - kCellPixelSize) / 2;

  if constexpr (kEnableAlphaBlending) {
    // ---- Alpha mode: chamfer-driven multi-tier blend ---------------
    // Two-pass chamfer: Manhattan distance from each fluid cell to the
    // nearest air cell (OOB treated as fluid via kBig so wall-adjacent
    // cells stay deep — walls don't generate a bright/halo edge).
    constexpr uint8_t kBig = 100;
    uint8_t depth[kVisibleCellsY][kVisibleCellsX];
    for (int y = 0; y < kVisibleCellsY; ++y) {
      for (int x = 0; x < kVisibleCellsX; ++x) {
        depth[y][x] = fluid[y][x] ? kBig : 0;
      }
    }
    for (int y = 0; y < kVisibleCellsY; ++y) {
      for (int x = 0; x < kVisibleCellsX; ++x) {
        if (depth[y][x] == 0) continue;
        const uint8_t up = (y > 0) ? depth[y - 1][x] : kBig;
        const uint8_t lt = (x > 0) ? depth[y][x - 1] : kBig;
        const uint8_t m  = up < lt ? up : lt;
        if (m + 1 < depth[y][x]) depth[y][x] = m + 1;
      }
    }
    for (int y = kVisibleCellsY - 1; y >= 0; --y) {
      for (int x = kVisibleCellsX - 1; x >= 0; --x) {
        if (depth[y][x] == 0) continue;
        const uint8_t dn = (y < kVisibleCellsY - 1) ? depth[y + 1][x] : kBig;
        const uint8_t rt = (x < kVisibleCellsX - 1) ? depth[y][x + 1] : kBig;
        const uint8_t m  = dn < rt ? dn : rt;
        if (m + 1 < depth[y][x]) depth[y][x] = m + 1;
      }
    }

    // Density-driven alpha jitter for the "granular" body texture.
    constexpr int kDensityJitter = 24;

    // Layer shallow→deep so deeper cells layer on top at overlap regions.
    for (int tier = 1; tier < kDepthTiers; ++tier) {
      const uint16_t color    = kDepthColors[tier];
      const int     baseAlpha = kDepthAlpha[tier];
      for (int y = 0; y < kVisibleCellsY; ++y) {
        for (int x = 0; x < kVisibleCellsX; ++x) {
          const uint8_t d = depth[y][x];
          if (d == 0) continue;
          const uint8_t t = d >= kDepthTiers ? kDepthTiers - 1 : d;
          if (t != tier) continue;
          const uint8_t count = grid[y][x];
          int a = baseAlpha;
          if      (count == 1) a -= kDensityJitter;
          else if (count >= 3) a += kDensityJitter;
          if (a < 0)   a = 0;
          if (a > 255) a = 255;
          sprite_.fillRectAlpha(xOff + x * kCellPixelSize - expand,
                                yOff + y * kCellPixelSize - expand,
                                kCellRenderSize, kCellRenderSize,
                                static_cast<uint8_t>(a), color);
        }
      }
    }
  } else {
    // ---- Opaque mode: rim + body + sparkle -------------------------
    // Binary rim test: a fluid cell with any non-fluid 4-neighbor is
    // rim. OOB treated as fluid so wall-adjacent cells stay body, not
    // rim — walls don't glow.
    uint8_t isRim[kVisibleCellsY][kVisibleCellsX];
    for (int y = 0; y < kVisibleCellsY; ++y) {
      for (int x = 0; x < kVisibleCellsX; ++x) {
        if (!fluid[y][x]) { isRim[y][x] = 0; continue; }
        const uint8_t up = (y > 0)                  ? fluid[y - 1][x] : 1;
        const uint8_t dn = (y < kVisibleCellsY - 1) ? fluid[y + 1][x] : 1;
        const uint8_t lt = (x > 0)                  ? fluid[y][x - 1] : 1;
        const uint8_t rt = (x < kVisibleCellsX - 1) ? fluid[y][x + 1] : 1;
        isRim[y][x] = (up && dn && lt && rt) ? 0 : 1;
      }
    }

    // Pass 1: body fill (non-rim fluid cells).
    for (int y = 0; y < kVisibleCellsY; ++y) {
      for (int x = 0; x < kVisibleCellsX; ++x) {
        if (!fluid[y][x] || isRim[y][x]) continue;
        sprite_.fillRect(xOff + x * kCellPixelSize - expand,
                         yOff + y * kCellPixelSize - expand,
                         kCellRenderSize, kCellRenderSize, kOpaqueBodyColor);
      }
    }

    // Pass 2: hash-driven sparkle dots inside body. Hash includes a
    // wall-clock frame index so the pattern advances frame-to-frame
    // and gives the "particle flicker" texture from the reference.
    const uint32_t fIdx = static_cast<uint32_t>(millis() / kOpaqueSparkleFrameMs);
    for (int y = 0; y < kVisibleCellsY; ++y) {
      for (int x = 0; x < kVisibleCellsX; ++x) {
        if (!fluid[y][x] || isRim[y][x]) continue;
        uint32_t h = (static_cast<uint32_t>(x) * 73856093u)
                   ^ (static_cast<uint32_t>(y) * 19349663u)
                   ^ (fIdx                     * 83492791u);
        if ((h % 100u) >= static_cast<uint32_t>(kOpaqueSparkleChancePct)) continue;
        const uint32_t h2 = h * 2654435761u;
        const int slack = kCellPixelSize - kOpaqueSparkleSize;
        const int sx = (slack > 0) ? static_cast<int>((h2 >> 16) % slack) : 0;
        const int sy = (slack > 0) ? static_cast<int>((h2 >> 8)  % slack) : 0;
        sprite_.fillRect(xOff + x * kCellPixelSize + sx,
                         yOff + y * kCellPixelSize + sy,
                         kOpaqueSparkleSize, kOpaqueSparkleSize,
                         kOpaqueSparkColor);
      }
    }

    // Pass 3: rim cells on top so the bright air-interface dominates
    // and the 1-px expand softly bleeds into adjacent body cells.
    for (int y = 0; y < kVisibleCellsY; ++y) {
      for (int x = 0; x < kVisibleCellsX; ++x) {
        if (!fluid[y][x] || !isRim[y][x]) continue;
        sprite_.fillRect(xOff + x * kCellPixelSize - expand,
                         yOff + y * kCellPixelSize - expand,
                         kCellRenderSize, kCellRenderSize, kOpaqueRimColor);
      }
    }
  }

  sprite_.pushSprite(0, 0);
}

}  // namespace fluidsim
