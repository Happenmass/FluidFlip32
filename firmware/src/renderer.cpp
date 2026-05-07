#include "renderer.h"

#include <algorithm>

namespace fluidsim {

// Density-tiered color palette (RGB565). Indexed by particle count per cell,
// clamped at 4. count=1 is the settled-body color (most common tier): clearly
// cyan with only a hint of background. Higher counts add brightness, count≥4
// gets a pale-white highlight for splash cores. count=0 stays bg so empty
// cells in the fluid look like genuine air gaps.
//
// Computed with bg=(R=0, G=0, B=16/31), cyan=(R=0, G=63/63, B=31/31).
constexpr uint16_t kDensityColors[5] = {
  0x0010,  // 0: deep blue background
  0x059A,  // 1: ~70% cyan (settled body)
  0x06BD,  // 2: ~85% cyan
  0x07FF,  // 3: full cyan (compressed)
  0x57FF,  // 4+: pale highlight (splash core, R=10/31 added)
};

// Blend two RGB565 pixels: result = fg*alpha + bg*(255-alpha), per channel.
// Uses >>8 instead of /255 — fast and visually indistinguishable.
static inline uint16_t blend565(uint16_t bg, uint16_t fg, uint8_t alpha) {
  uint8_t inv = 255 - alpha;
  uint16_t bg_r = (bg >> 11) & 0x1F;
  uint16_t bg_g = (bg >> 5)  & 0x3F;
  uint16_t bg_b =  bg        & 0x1F;
  uint16_t fg_r = (fg >> 11) & 0x1F;
  uint16_t fg_g = (fg >> 5)  & 0x3F;
  uint16_t fg_b =  fg        & 0x1F;
  uint16_t r = (fg_r * alpha + bg_r * inv) >> 8;
  uint16_t g = (fg_g * alpha + bg_g * inv) >> 8;
  uint16_t b = (fg_b * alpha + bg_b * inv) >> 8;
  return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

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

  uint16_t* buf = static_cast<uint16_t*>(sprite_.getBuffer());
  const int spriteW = sprite_.width();
  const int spriteH = sprite_.height();

  for (int y = 0; y < kVisibleCellsY; ++y) {
    for (int x = 0; x < kVisibleCellsX; ++x) {
      uint8_t count = grid[y][x];
      if (count == 0) continue;
      uint8_t tier = count >= 4 ? 4 : count;
      uint16_t color = kDensityColors[tier];

      const int x0 = xOff + x * kCellPixelSize;
      const int y0 = yOff + y * kCellPixelSize;
      const int x1 = std::min(x0 + kCellRenderSize, spriteW);
      const int y1 = std::min(y0 + kCellRenderSize, spriteH);
      const int xs = std::max(x0, 0);
      const int ys = std::max(y0, 0);

      for (int py = ys; py < y1; ++py) {
        uint16_t* row = buf + py * spriteW;
        for (int px = xs; px < x1; ++px) {
          row[px] = blend565(row[px], color, kFluidAlpha);
        }
      }
    }
  }
  sprite_.pushSprite(0, 0);
}

}  // namespace fluidsim
