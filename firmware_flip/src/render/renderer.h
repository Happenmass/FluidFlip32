#pragma once

#include <array>

#include <M5Unified.h>

#include "../config.h"
#include "../fluid/mac_grid.h"

namespace flip {

// LovyanGFX sprite-based renderer. Owns a full-screen 16-bit sprite that
// every draw() call rebuilds and pushes to the LCD in one transfer.
//
// Stage 3 takes a *snapshot* of the per-cell particle count rather than a
// live FlipSolver reference: callers grab the snapshot under whatever lock
// guards the physics task's recountForRender(), then hand the buffer in
// here so the renderer can iterate without worrying about concurrent
// writes from core 1.
class Renderer {
 public:
  Renderer();
  void begin();
  void draw(const float* density, int num_particles, float fps);

 private:
  M5Canvas sprite_;
  bool     ready_;

  // Per-cell EMA-smoothed density, persists across frames. Owned by the
  // render task (core 0) only — never written from physics — so it
  // doesn't need lock protection. Initial frame seeds straight from the
  // input snapshot via ema_initialized_.
  std::array<float, MacGrid::kCSize> ema_density_{};
  bool                               ema_initialized_ = false;
};

}  // namespace flip
