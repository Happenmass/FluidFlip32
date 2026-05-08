#include "imu_reader.h"

#include <M5Unified.h>

namespace flip {

ImuReader::ImuReader() = default;

bool ImuReader::begin() {
  // M5.begin() already initialized the IMU when cfg.internal_imu was set.
  // We just probe by attempting one read — if the first sample succeeds the
  // BMI270 is alive on the I²C bus.
  float ax, ay, az;
  ok_ = M5.Imu.getAccel(&ax, &ay, &az);
  if (ok_) { ax_ = ax; ay_ = ay; az_ = az; }
  return ok_;
}

void ImuReader::poll(float& gx, float& gy) {
  float ax = ax_, ay = ay_, az = az_;
  if (M5.Imu.getAccel(&ax, &ay, &az)) {
    ax_ = ax; ay_ = ay; az_ = az;
    ok_ = true;
  }

  // Empirical M5StickS3 mapping (rotation=1 landscape), verified on the
  // existing firmware's input layer: IMU X axis points opposite to screen
  // +x, IMU Y axis aligns with screen +y. The acceleration reading is the
  // reaction force, which already has the correct sign for "gravity pulls
  // particles in this direction" once we apply that remap.
  const float gx_screen = -ax_;
  const float gy_screen =  ay_;
  gx = gx_screen * kGravityMagnitude;
  gy = gy_screen * kGravityMagnitude;
}

}  // namespace flip
