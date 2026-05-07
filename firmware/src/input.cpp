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

  // StickS3 in landscape rotation=1 (verified empirically on device):
  //   IMU X axis aligns with screen-Y (vertical / LCD short edge).
  //   IMU Y axis aligns with screen-X (horizontal / LCD long edge).
  // So screen gravity components come from the swapped raw values.
  // Signs determined by hardware testing — flip just one if a single
  // direction feels reversed.
  float gx_screen = ay;
  float gy_screen = ax;
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
