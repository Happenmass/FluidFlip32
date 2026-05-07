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

  // IMU read. M5Unified returns m/s² when IMU support is internal_unit_g = false,
  // but the typical Accel API returns g-units; consult the actual return value.
  float ax = lastAx_, ay = lastAy_, az = lastAz_;
  if (M5.Imu.getAccel(&ax, &ay, &az)) {
    lastAx_ = ax; lastAy_ = ay; lastAz_ = az;
  }

  // Axis remap: M5StickC family with rotation=1 (landscape) typically has
  // screen-X = +imuY, screen-Y = -imuX (verify in Task 15 bringup).
  // For now, ship a sane default; the bringup task will Serial-log raw values
  // and fix signs.
  float gx_screen =  ay;
  float gy_screen = -ax;
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
  st.btnB_pressed = M5.BtnB.wasPressed();
  st.btnB_long    = M5.BtnB.wasReleasedAfterHold();
  return st;
}

}  // namespace fluidsim
