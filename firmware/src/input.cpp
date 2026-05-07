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
  //   raw ax → screen-X (horizontal), raw ay → screen-Y (vertical, +y = down).
  // Held upright in landscape, raw ay ≈ +1g and raw ax ≈ 0, so particles
  // settle along screen-Y as expected. If a single tilt direction feels
  // reversed on hardware, flip the sign on just that line.
  //
  // (Note: claude-desktop-buddy swaps ax/ay on StickS3 for its 1↔3
  // orientation-detection convention, not for 2D gravity mapping. Don't
  // copy that swap here — it rotates the gravity vector by 90°.)
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
