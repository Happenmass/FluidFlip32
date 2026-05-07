#pragma once

#include <M5Unified.h>

#include "fluid_sim.h"

namespace fluidsim {

class Renderer {
 public:
  Renderer();
  void begin();
  void draw(const Scene::OutputGrid& grid);

 private:
  M5Canvas sprite_;
  bool initialized_;
};

}  // namespace fluidsim
