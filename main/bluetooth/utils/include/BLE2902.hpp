#pragma once

#include "sdkconfig.h"
#include "BLEDescriptor.hpp"
#include "esp_gatt_defs.h"

/**
 * @brief Descriptor for Client Characteristic Configuration.
 *
 * This is a convenience descriptor for the Client Characteristic Configuration which has a UUID of 0x2902.
 *
 * See also:
 * https://www.bluetooth.com/specifications/assigned-numbers/
 */

class BLE2902: public BLEDescriptor {
public:
	BLE2902();
	bool getNotifications() const;
	bool getIndications() const;
	void setNotifications(bool flag);
	void setIndications(bool flag);
};
