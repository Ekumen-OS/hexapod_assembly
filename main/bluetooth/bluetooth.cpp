#include "bluetooth.h"

#include "bt_nimble.h"
#include "gatt_svrc.h"
#include "host/ble_hs.h"

namespace {
bool new_data_from_bt = false;
char bt_data;

// ID of service: b2bbc642-46da-11ed-b878-0242ac120002
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0x78, 0xb8, 0xed, 0x11,
                     0xda, 0x46, 0x42, 0xc6, 0xbb, 0xb2);

// ID of characteristic: c9af9c76-46de-11ed-b878-0242ac120002
static const ble_uuid128_t gatt_svr_chr_one_uuid =
    BLE_UUID128_INIT(0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0x78, 0xb8, 0xed, 0x11,
                     0xde, 0x46, 0x76, 0x9c, 0xaf, 0xc9);

const uint16_t kMinWriteLength =
    1;  // Minimum length the client can write to a characterstic
const uint16_t kMaxWriteLength =
    700;  // Maximum length the client can write to a characterstic

static int bt_event_callback(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt,
                                   void *arg) {
  int rc;
  char read_value[50] =
      "I am characteristic One value";  // When you read characteristic one,
                                        // you get this
  char received_value
      [500];  // When you write to characteristic one, you set value of this

  switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:  // In case user accessed this
                                       // characterstic to read its value,
                                       // bellow lines will execute
      rc = os_mbuf_append(ctxt->om, &read_value,
                          sizeof read_value);
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:  // In case user accessed this
                                        // characterstic to write, bellow lines
                                        // will executed.
      rc = gatt_svr_chr_write(
          ctxt->om, kMinWriteLength, kMaxWriteLength, &received_value,
          NULL); 
      printf("Received=%s\n",
             received_value);  // Print the received value

      // Set new data.
      bt_data = received_value[0];
      new_data_from_bt = true;

      memset(
          received_value, '\0',
          sizeof received_value);  // Empty string for next
                                                      // value to receive

      return rc;

    default:
      assert(0);
      return BLE_ATT_ERR_UNLIKELY;
  }

  assert(0);
  return BLE_ATT_ERR_UNLIKELY;
}

static struct ble_gatt_svc_def gatt_svr_svcs[] = {
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &gatt_svr_svc_uuid.u,
    .characteristics =
      (struct ble_gatt_chr_def[]){
      {
        .uuid = &gatt_svr_chr_one_uuid.u,  //!! UUID as given above
        .access_cb =
            bt_event_callback,  //!! Callback function. When
                                      //!ever this characrstic will
                                      //!be accessed by user, this
                                      //!function will execute
        .flags = BLE_GATT_CHR_F_READ |
                  BLE_GATT_CHR_F_WRITE,  //!! flags set permissions.
                                        //!In this case User can
                                        //!read This characterstic
                                        //!and can write to it
      },
      {
          0,
      }
    },
  },
  {
      0,
  },
};

} // Anonymous namespace.

Bluetooth::Bluetooth()
    : gatt_svr_svcs_(gatt_svr_svcs) {
  init_nimble(gatt_svr_svcs_);
}

bool Bluetooth::hasNewData() {
  return new_data_from_bt;
}

int Bluetooth::getChar(char& out_data) {
  if (!new_data_from_bt) {
    out_data = '\0';
    return 0;
  }

  out_data = bt_data;
  new_data_from_bt = false;
  return 0;

}