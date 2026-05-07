#include <M5Unified.h>

#include "fluid_sim.h"
#include "fluid_sim_config.h"
#include "input.h"
#include "renderer.h"

using namespace fluidsim;

static Scene             gScene;
static Renderer          gRenderer;
static InputController   gInput;

static uint32_t gFpsLastReport = 0;
static uint32_t gFpsFrames     = 0;

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = true;
  M5.begin(cfg);
  Serial.begin(115200);
  delay(200);
  Serial.println("[boot] M5StickS3 FluidSim starting");
  M5.BtnB.setHoldThresh(1000);

  gRenderer.begin();
  gInput.begin();
  gScene.setup(kDefaultParticles);
  gFpsLastReport = millis();
}

void loop() {
  M5.update();
  InputState in = gInput.poll();

  gScene.setGravity(in.gravityX, in.gravityY);

  if (in.btnA_pressed) {
    gScene.applyRadialImpulse(kImpulseCenterX, kImpulseCenterY,
                              kImpulseStrength, kImpulseRadius);
  }
  if (in.btnB_long) {
    gScene.setup(kDefaultParticles);
  } else if (in.btnB_pressed) {
    gScene.particleAdd(100, kMaxParticles);
  }

  for (int i = 0; i < kSubstepsPerFrame; ++i) {
    gScene.simulate();
  }
  Scene::OutputGrid grid;
  gScene.getOutput(grid);
  gRenderer.draw(grid);

  gFpsFrames++;
  uint32_t now = millis();
  if (now - gFpsLastReport >= 1000) {
    Serial.printf("[fps] %u frames in %u ms (particles=%d)\n",
                  gFpsFrames, now - gFpsLastReport, gScene.numParticles());
    gFpsFrames = 0;
    gFpsLastReport = now;
  }
}
