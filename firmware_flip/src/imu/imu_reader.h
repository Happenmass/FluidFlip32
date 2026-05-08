#pragma once

#include "../config.h"

namespace flip {

// BMI270 accelerometer reader, talking to the chip via M5Unified's bundled
// driver (`M5.Imu` returns a unified accel API regardless of the underlying
// MPU/BMI part). All values returned from this module are in screen-space
// solver units; raw sensor values are exposed only for axis calibration.
class ImuReader {
 public:
  ImuReader();

  // Bring up M5.Imu. Caller is responsible for setting cfg.internal_imu=true
  // in M5.config() *before* M5.begin(); this just verifies the IMU answered.
  bool begin();

  // Sample the IMU and produce a gravity vector in solver units (cells / s²).
  //   gx > 0 → pulls particles right (screen +x).
  //   gy > 0 → pulls particles down  (screen +y, matches solver convention).
  // If the read fails this frame, returns the last successful sample so the
  // sim doesn't hiccup.
  void poll(float& gx, float& gy);

  // Last raw accel read (g-units, IMU-native axes — NOT remapped). Useful
  // for axis calibration: print these while tilting the device, confirm
  // signs match expectations before trusting the screen-space mapping.
  void rawAccel(float& ax, float& ay, float& az) const {
    ax = ax_; ay = ay_; az = az_;
  }

  bool ok() const { return ok_; }

 private:
  float ax_ = 0.f, ay_ = 0.f, az_ = 1.f;  // last raw read (g-units)
  bool  ok_ = false;
};

}  // namespace flip
