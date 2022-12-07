/*************************************************** 
  This code has been take from here: https://github.com/Zeni241/ESP32-NimbleBLE-For-Dummies and then following changes were made:
  - Create one headerfile per each cpp file.

  You can find these changes here: https://gitlab.com/ekumenlabs/ekumembers/ekuthon2022/hexapod_assembly/-/commit/541c326250f34d0bebe1a63bf07030a959bde528
  I have requested the author of this code to add a license to it: https://github.com/Zeni241/ESP32-NimbleBLE-For-Dummies/issues/2
 ****************************************************/

#pragma once
#include "gatt_svrc.h"

void init_nimble(const ble_gatt_svc_def* gatt_svr_svcs);
