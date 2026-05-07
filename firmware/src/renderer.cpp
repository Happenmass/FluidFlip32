#include "renderer.h"

namespace fluidsim {

// Colors tuned to match ref.jpg: deep ocean blue background, cyan fluid body
// with a brighter highlight on the surface (cells whose top neighbor is air).
constexpr uint16_t kColorBg           = 0x0010;  // deep blue
constexpr uint16_t kColorFluidBody    = 0x07FF;  // cyan
constexpr uint16_t kColorFluidSurface = 0xAFFF;  // pale cyan (surface highlight)

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

  // Center the visible grid vertically. 16 rows × 8 px = 128 px → 7 px letterbox.
  const int xOff = 0;
  const int yOff = (M5.Display.height() - kVisibleCellsY * kCellPixelSize) / 2;

  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      if (!grid[y][x]) continue;
      // Surface cell: has air immediately above (or sits on the visible top row).
      // This gives the fluid a bright "waterline" highlight matching the ref.
      bool surface = (y == 0) || !grid[y - 1][x];
      uint16_t color = surface ? kColorFluidSurface : kColorFluidBody;
      sprite_.fillRect(xOff + x * kCellPixelSize,
                       yOff + y * kCellPixelSize,
                       kCellPixelSize, kCellPixelSize, color);
    }
  }
  sprite_.pushSprite(0, 0);
}

}  // namespace fluidsim
