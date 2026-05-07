#include "input.h"
#include "fluid_sim_config.h"
#include <cmath>

namespace fluidsim {

InputController::InputController()
    : shakeFrames_(0), lastAx_(0.0f), lastAy_(0.0f), lastAz_(1.0f) {}

void InputController::begin() {
  // M5Unified initializes IMU automatically when M5.config().internal_imu = true.
}

InputState InputController::poll() {
  InputState st{};

  // IMU read in g-units.
  float ax = lastAx_, ay = lastAy_, az = lastAz_;
  if (M5.Imu.getAccel(&ax, &ay, &az)) {
    lastAx_ = ax; lastAy_ = ay; lastAz_ = az;
  }

  // StickS3's IMU is mounted with the device's long edge along the IMU's
  // Y axis (StickC Plus family had it along X). Swap so the rest of the
  // mapping can use StickC Plus convention. Reference:
  // ../claude-desktop-buddy/src/main.cpp:392-399
  if (M5.getBoard() == m5::board_t::board_M5StickS3) {
    float t = ax; ax = ay; ay = t;
  }

  // Landscape (rotation=1) mapping after the swap:
  //   screen-X (long edge of LCD, horizontal) = +ax (raw long-edge axis)
  //   screen-Y (short edge of LCD, vertical)  = +ay (raw short-edge axis)
  // Sign correctness depends on physical mounting; flip either line if
  // tilt direction feels inverted on hardware.
  float gx_screen = ax;
  float gy_screen = ay;
  st.gravityX = gx_screen * kGravityScale;
  st.gravityY = gy_screen * kGravityScale;

  float mag = std::sqrt(ax * ax + ay * ay + az * az);
  if (mag > kShakeMagnitudeG) {
    shakeFrames_++;
  } else if (shakeFrames_ > 0) {
    shakeFrames_--;
  }
  if (shakeFrames_ > kShakeFramesNeeded) {
    st.shakeTriggered = true;
    shakeFrames_ = 0;
  }

  st.btnA_pressed = M5.BtnA.wasPressed();
  st.btnB_pressed = M5.BtnB.wasClicked();
  st.btnB_long    = M5.BtnB.wasReleasedAfterHold();
  return st;
}

}  // namespace fluidsim
