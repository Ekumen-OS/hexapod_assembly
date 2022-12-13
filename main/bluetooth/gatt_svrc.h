/*************************************************** 
  MIT License

  Copyright (c) 2022 Zeni241

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
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
