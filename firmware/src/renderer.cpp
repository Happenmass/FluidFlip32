#include "renderer.h"

namespace fluidsim {

// Density-tiered color palette (RGB565). Indexed by particle count per cell,
// clamped at 4. Linearly blends from deep-blue background → cyan, with a
// brighter highlight for compressed/dense regions. This gives the fluid the
// "crystal clear seawater" look from ref.jpg: surface cells (count=1) read as
// translucent, settled body (count=2-3) as cyan, splash cores (count≥4) bright.
//
// Computed with bg=(R=0, G=0, B=16/31), cyan=(R=0, G=63/63, B=31/31).
constexpr uint16_t kDensityColors[5] = {
  0x0010,  // 0: deep blue background
  0x0214,  // 1: ~25% cyan (translucent water surface)
  0x03F8,  // 2: ~50% cyan (settled body)
  0x05FB,  // 3: ~75% cyan
  0x07FF,  // 4+: full cyan (compressed)
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
  sprite_.fillScreen(kDensityColors[0]);

  // Center the visible grid vertically. 16 rows × 8 px = 128 px → 7 px letterbox.
  const int xOff = 0;
  const int yOff = (M5.Display.height() - kVisibleCellsY * kCellPixelSize) / 2;

  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      uint8_t count = grid[y][x];
      if (count == 0) continue;  // background already painted
      uint8_t tier = count >= 4 ? 4 : count;
      sprite_.fillRect(xOff + x * kCellPixelSize,
                       yOff + y * kCellPixelSize,
                       kCellPixelSize, kCellPixelSize, kDensityColors[tier]);
    }
  }
  sprite_.pushSprite(0, 0);
}

}  // namespace fluidsim
