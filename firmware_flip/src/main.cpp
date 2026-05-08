#include <Arduino.h>
#include <M5Unified.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>

#include "config.h"
#include "fluid/flip_solver.h"
#include "fluid/mac_grid.h"
#if FLIP_USE_IMU
#  include "imu/imu_reader.h"
#endif
#include "render/renderer.h"

using namespace flip;

namespace {

FlipSolver gSolver;
Renderer   gRenderer;
#if FLIP_USE_IMU
ImuReader  gImu;
#endif

// ── Thread plumbing ──────────────────────────────────────────────────────
// Mutex protects: gSolver.recountForRender() (writer, on physics task) and
// snapshotting gSolver.particleCounts() (reader, on render task). Held only
// for the brief publish/copy windows — not during the long physics step or
// the long sprite push, so the two cores actually run in parallel.
SemaphoreHandle_t gCountMutex = nullptr;

// Gravity vector handed from render-task IMU read → physics-task step.
// std::atomic<float> on ESP32-S3 is lock-free; readers always see a
// consistent value without needing a mutex.
std::atomic<float> gGravityX{0.f};
std::atomic<float> gGravityY{kGravityMagnitude};

// FPS smoothed across the last second, written by render task.
std::atomic<float> gRenderFps{0.f};

// ── Button-driven control plumbing ────────────────────────────────────────
// Both buttons mutate gSolver state (particles_ vector / reset). Since
// that vector is owned by the physics task on core 1, the main loop only
// signals intent via atomics; the physics task drains them at the head
// of each tick. This keeps the producer/consumer split clean — no extra
// mutex around the particle vector.
//
//   BtnA held       → 10 Hz pulse increments gInjectPending
//   BtnB pressed    → gResetRequested := true (one-shot)
std::atomic<int>  gInjectPending{0};
std::atomic<bool> gResetRequested{false};
uint32_t          gLastInjectMs = 0;

// FPS bookkeeping (1 Hz) — rendered on the sprite, also dumped to serial.
uint32_t gFpsLastReportMs = 0;
uint32_t gFpsFrames       = 0;

#if FLIP_USE_IMU
constexpr uint32_t kCalibrationWindowMs = 5000;
constexpr uint32_t kCalibrationPeriodMs = 200;
uint32_t           gBootMs               = 0;
uint32_t           gCalibLastPrintMs     = 0;
#endif

// ── Physics task (pinned to core 1) ──────────────────────────────────────
// Runs a fixed-dt accumulator like the single-core version did, but
// independently of the render loop. After each batch of steps, briefly
// holds the count mutex to refresh the render-visible particle_count.
void physicsTask(void* /*arg*/) {
  uint32_t lastUs       = micros();
  float    accumulator  = 0.f;

  for (;;) {
    // ── Drain button-driven state changes (BtnB reset, BtnA injects) ──
    // Reset has higher priority — clearing the solver also makes any
    // stale pending-inject counter meaningless, so we zero it too. Both
    // happen *between* physics steps, not in the middle of one, so the
    // particles_ vector is in a consistent state when the next step
    // starts integrating.
    if (gResetRequested.exchange(false, std::memory_order_relaxed)) {
      gSolver.reset(kInitParticles);
      gInjectPending.store(0, std::memory_order_relaxed);
      accumulator = 0.f;
    }
    int pending = gInjectPending.exchange(0, std::memory_order_relaxed);
    if (pending > 0) {
      // Sample current gravity once per drain cycle so all the queued
      // injections share the same "up" axis (matters at most for ~few
      // ms of flight time — gravity barely changes between consecutive
      // 100ms button pulses).
      const float curGx = gGravityX.load(std::memory_order_relaxed);
      const float curGy = gGravityY.load(std::memory_order_relaxed);
      while (pending-- > 0 && gSolver.injectFromTop(curGx, curGy)) {
        // injectFromTop() returns false at cap; loop early-exits there.
      }
    }

    const uint32_t nowUs = micros();
    float          dt    = (nowUs - lastUs) * 1e-6f;
    lastUs               = nowUs;
    if (dt > kMaxFrameDt) dt = kMaxFrameDt;
    accumulator += dt;

    bool stepped = false;
    while (accumulator >= kFixedDt) {
      const float gx = gGravityX.load(std::memory_order_relaxed);
      const float gy = gGravityY.load(std::memory_order_relaxed);
      gSolver.step(kFixedDt, gx, gy);
      accumulator -= kFixedDt;
      stepped = true;
    }

    if (stepped) {
      // Publish particle counts for the render task. Brief lock — only the
      // 512-byte recount runs under the mutex, not the long physics step.
      xSemaphoreTake(gCountMutex, portMAX_DELAY);
      gSolver.recountForRender();
      xSemaphoreGive(gCountMutex);
    }

    // Yield so FreeRTOS housekeeping on core 1 can run; 1 tick == ~1 ms.
    vTaskDelay(1);
  }
}

}  // namespace

void setup() {
  auto cfg = M5.config();
#if FLIP_USE_IMU
  cfg.internal_imu = true;
#else
  cfg.internal_imu = false;
#endif
  M5.begin(cfg);
  Serial.begin(115200);
  delay(150);
  Serial.printf("[boot] firmware_flip Stage %d starting\n", FLIP_STAGE);
  Serial.printf("[cfg]  grid=%dx%d, particles=%d, dt=%.4f s, imu=%d\n",
                kVisibleCellsX, kVisibleCellsY, kInitParticles,
                kFixedDt, FLIP_USE_IMU);

#if FLIP_USE_IMU
  if (gImu.begin()) {
    Serial.println("[imu]  BMI270 online — entering 5 s axis-calibration window");
  } else {
    Serial.println("[imu]  WARNING: getAccel() failed; gravity will fall back to last sample");
  }
  gBootMs = millis();
#endif

  gRenderer.begin();
  gSolver.reset(kInitParticles);

  // Prime the render-visible count so the first frame doesn't draw a blank
  // screen before the physics task has run a step.
  gSolver.recountForRender();

  gCountMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(physicsTask, "physics", kPhysicsTaskStack,
                          nullptr, kPhysicsTaskPrio, nullptr,
                          kPhysicsTaskCore);

  gFpsLastReportMs = millis();
}

void loop() {
  M5.update();

  // ── Buttons → physics-task signals ────────────────────────────────────
  // BtnA held: paced injection at kInjectIntervalMs (10 Hz default).
  // Increment a counter the physics task drains. isPressed() is the
  // continuous "currently held" state, so releasing the button stops
  // the pulses naturally.
  if (M5.BtnA.isPressed()) {
    const uint32_t nowMs = millis();
    if (nowMs - gLastInjectMs >= kInjectIntervalMs) {
      gInjectPending.fetch_add(1, std::memory_order_relaxed);
      gLastInjectMs = nowMs;
    }
  } else {
    // While button is up, keep gLastInjectMs aligned to "now − interval"
    // so the very first press doesn't either fire instantly (if stale)
    // or wait a full interval. This way a press always fires within a
    // few main-loop ticks but never doubles up.
    gLastInjectMs = millis() - kInjectIntervalMs;
  }
  // BtnB pressed (edge-triggered): one-shot reset request.
  if (M5.BtnB.wasPressed()) {
    gResetRequested.store(true, std::memory_order_relaxed);
  }

  // ── Read IMU and publish gravity to physics task ──────────────────────
  float gx, gy;
#if FLIP_USE_IMU
  gImu.poll(gx, gy);
#else
  gx = 0.f;
  gy = kGravityMagnitude;
#endif
  gGravityX.store(gx, std::memory_order_relaxed);
  gGravityY.store(gy, std::memory_order_relaxed);

  // ── Snapshot density field under mutex ────────────────────────────────
  // Stack-allocated copy (lives in BSS via static) so the actual rendering
  // work happens lock-free. Float density × 612 cells = ~2.4 KB.
  static std::array<float, MacGrid::kCSize> snapshot;
  xSemaphoreTake(gCountMutex, portMAX_DELAY);
  snapshot = gSolver.densities();             // ~2.4-KB memcpy
  const int numParticlesSnapshot = gSolver.numParticles();
  xSemaphoreGive(gCountMutex);

  // ── Render (no lock — works on local snapshot) ────────────────────────
  gRenderer.draw(snapshot.data(), numParticlesSnapshot,
                 gRenderFps.load(std::memory_order_relaxed));

#if FLIP_USE_IMU
  // ── IMU calibration print (first 5 s only) ────────────────────────────
  const uint32_t bootElapsed = millis() - gBootMs;
  if (bootElapsed < kCalibrationWindowMs &&
      millis() - gCalibLastPrintMs >= kCalibrationPeriodMs) {
    float ax, ay, az;
    gImu.rawAccel(ax, ay, az);
    Serial.printf("[imu] raw ax=%+.3f ay=%+.3f az=%+.3f  →  gx=%+.2f gy=%+.2f\n",
                  ax, ay, az, gx, gy);
    gCalibLastPrintMs = millis();
  }
#endif

  // ── FPS bookkeeping (1 Hz) ────────────────────────────────────────────
  ++gFpsFrames;
  const uint32_t nowMs = millis();
  if (nowMs - gFpsLastReportMs >= 1000) {
    const float fps = gFpsFrames * 1000.f /
                      static_cast<float>(nowMs - gFpsLastReportMs);
    gRenderFps.store(fps, std::memory_order_relaxed);
    Serial.printf("[fps] %.1f  (particles=%d, gx=%+.2f, gy=%+.2f)\n",
                  fps, numParticlesSnapshot, gx, gy);
    gFpsFrames       = 0;
    gFpsLastReportMs = nowMs;
  }
}
