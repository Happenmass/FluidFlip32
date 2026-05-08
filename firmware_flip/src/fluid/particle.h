#pragma once

namespace flip {

// Single fluid particle in solver-internal cell coordinates.
// Position (x, y) is in [1, kCellsX-1] × [1, kCellsY-1] (interior of MAC
// grid; the outer ring of cells is solid wall). Velocity is in cells / s.
struct Particle {
  float x, y;
  float vx, vy;
};

}  // namespace flip
