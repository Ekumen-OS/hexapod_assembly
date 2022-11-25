#include "hexapod.h"

extern "C" {
void app_main();
}

void app_main() {
  Hexapod hexapod;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    hexapod.moveForward();
    vTaskDelay(pdMS_TO_TICKS(1000));
    hexapod.moveBackwards();
  }
}
