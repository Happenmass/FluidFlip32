#include <M5Unified.h>

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  delay(200);
  Serial.println("[boot] M5StickS3 FluidSim starting");
  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_CYAN);
  M5.Display.setCursor(4, 4);
  M5.Display.print("FluidSim boot");
}

void loop() {
  M5.update();
  delay(100);
}
