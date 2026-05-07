#pragma once

#include <M5Unified.h>

namespace fluidsim {

struct InputState {
  float gravityX;     // sim cells / s²
  float gravityY;     // sim cells / s²
  bool  btnA_pressed;
  bool  btnB_pressed;
  bool  btnB_long;
};

class InputController {
 public:
  InputController();
  void begin();
  InputState poll();

 private:
  float lastAx_, lastAy_, lastAz_;
};

}  // namespace fluidsim
