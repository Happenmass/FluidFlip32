#pragma once

#include <M5Unified.h>

namespace fluidsim {

struct InputState {
  float gravityX;     // sim cells / s²
  float gravityY;     // sim cells / s²
  bool  shakeTriggered;
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
  int   shakeFrames_;
  float lastAx_, lastAy_, lastAz_;
};

}  // namespace fluidsim
