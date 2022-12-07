#include "hexapod.h"
#include "bluetooth/bluetooth.h"

extern "C" {
void app_main();
}

void app_main() {
  Hexapod hexapod;
  Bluetooth bt;  
  hexapod.stand();
//  hexapod.calibrateJoint();
/*
  vTaskDelay(pdMS_TO_TICKS(1000));
  hexapod.standUp();
  hexapod.stand();
  vTaskDelay(pdMS_TO_TICKS(3000));
  hexapod.sayHi();
  while (1) {
    hexapod.moveForward();
  }
*/

  char data;
  while(1) {
    if(bt.hasNewData()) {
      printf("if(bt.hasNewData())\n");
      bt.getChar(data);
      printf("Data: %c\n", data);

      switch (data)
      {
      case 'q':
        hexapod.sayHi();
        break;

      case 'w':
        hexapod.moveForward();
        break;

      case 'c':
        hexapod.standUp();
        break;

      case 'x':
        hexapod.stand();
        break;
      
      default:
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }



}
