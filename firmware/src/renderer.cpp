#include "renderer.h"

namespace fluidsim {

constexpr uint16_t kColorBg    = 0x0000;  // black
constexpr uint16_t kColorFluid = 0x07FF;  // cyan

Renderer::Renderer() : sprite_(&M5.Display), initialized_(false) {}

void Renderer::begin() {
  M5.Display.setRotation(1);                      // 240×135 landscape
  sprite_.setColorDepth(16);
  sprite_.createSprite(M5.Display.width(), M5.Display.height());
  initialized_ = true;
}

void Renderer::draw(const Scene::OutputGrid& grid) {
  if (!initialized_) return;
  sprite_.fillScreen(kColorBg);

  // 22 visible rows × 6 px = 132 px, leaving 3 px letterbox total.
  // Center vertically: y_offset = (135 - 132) / 2 = 1 (1 px top, 2 px bottom).
  const int xOff = 0;
  const int yOff = (M5.Display.height() - kVisibleCellsY * kCellPixelSize) / 2;

  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      if (!grid[y][x]) continue;
      sprite_.fillRect(xOff + x * kCellPixelSize,
                       yOff + y * kCellPixelSize,
                       kCellPixelSize, kCellPixelSize, kColorFluid);
    }
  }
  sprite_.pushSprite(0, 0);
}

}  // namespace fluidsim
