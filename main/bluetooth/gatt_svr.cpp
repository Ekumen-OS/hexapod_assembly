

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

extern uint16_t
    notification_handle;  //!! Whenever value of this variable is changed,
                          //!server send that value as notification to client
extern uint16_t conn_handle;

int gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                       void *dst, uint16_t *len) {
  uint16_t om_len;
  int rc;

  om_len = OS_MBUF_PKTLEN(om);
  if (om_len < min_len || om_len > max_len) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
  if (rc != 0) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  return 0;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
  char buf[BLE_UUID_STR_LEN];

  switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
      MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                  ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                  ctxt->svc.handle);
      break;

    case BLE_GATT_REGISTER_OP_CHR:
      MODLOG_DFLT(DEBUG,
                  "registering characteristic %s with "
                  "def_handle=%d val_handle=%d\n",
                  ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                  ctxt->chr.def_handle, ctxt->chr.val_handle);
      break;

    case BLE_GATT_REGISTER_OP_DSC:
      MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                  ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                  ctxt->dsc.handle);
      break;

    default:
      assert(0);
      break;
  }
}

// int gatt_svr_init()
int gatt_svr_init(const struct ble_gatt_svc_def *gatt_svr_svcs) {
  int rc;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  rc = ble_gatts_count_cfg(gatt_svr_svcs);
  if (rc != 0) {
    return rc;
  }

  rc = ble_gatts_add_svcs(gatt_svr_svcs);
  if (rc != 0) {
    return rc;
  }

  return 0;
}
