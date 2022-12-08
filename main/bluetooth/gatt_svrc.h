/*************************************************** 
  This code has been take from here: https://github.com/Zeni241/ESP32-NimbleBLE-For-Dummies and then following changes were made:
  - Create one headerfile per each cpp file.

  You can find these changes here: https://gitlab.com/ekumenlabs/ekumembers/ekuthon2022/hexapod_assembly/-/commit/541c326250f34d0bebe1a63bf07030a959bde528
  I have requested the author of this code to add a license to it: https://github.com/Zeni241/ESP32-NimbleBLE-For-Dummies/issues/2
 ****************************************************/

#pragma once

#include <stdbool.h>

#include "modlog/modlog.h"
#include "nimble/ble.h"

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;
struct ble_gatt_svc_def;

/** GATT server. */
#define GATT_SVR_SVC_ALERT_UUID 0x1811
#define GATT_SVR_CHR_SUP_NEW_ALERT_CAT_UUID 0x2A47
#define GATT_SVR_CHR_NEW_ALERT 0x2A46
#define GATT_SVR_CHR_SUP_UNR_ALERT_CAT_UUID 0x2A48
#define GATT_SVR_CHR_UNR_ALERT_STAT_UUID 0x2A45
#define GATT_SVR_CHR_ALERT_NOT_CTRL_PT 0x2A44

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_svr_init(const struct ble_gatt_svc_def *gatt_svr_svcs);
int gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                       void *dst, uint16_t *len);
