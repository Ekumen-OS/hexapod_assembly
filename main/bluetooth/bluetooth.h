#pragma once
#include "gatt_svrc.h"

class Bluetooth {
 public:
  Bluetooth();
  
  bool hasNewData();
  int getChar(char& out_data);

 private:
  const ble_gatt_svc_def* gatt_svr_svcs_;
};
