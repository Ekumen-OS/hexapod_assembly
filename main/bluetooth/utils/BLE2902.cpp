#include "BLE2902.hpp"

BLE2902::BLE2902() : BLEDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_CLIENT_CONFIG)) {
	uint8_t data[2] = { 0, 0 };
	setValue(data, 2);
}

/**
 * @brief Get the notifications value.
 * @return The notifications value.  True if notifications are enabled and false if not.
 */
bool BLE2902::getNotifications() const {
	return (getValue()[0] & (1 << 0)) != 0;
}

/**
 * @brief Get the indications value.
 * @return The indications value.  True if indications are enabled and false if not.
 */
bool BLE2902::getIndications() const {
	return (getValue()[0] & (1 << 1)) != 0;
}

/**
 * @brief Set the indications flag.
 * @param [in] flag The indications flag.
 */
void BLE2902::setIndications(bool flag) {
	uint8_t *pValue = getValue();
	if (flag) pValue[0] |= 1 << 1;
	else pValue[0] &= ~(1 << 1);
}

/**
 * @brief Set the notifications flag.
 * @param [in] flag The notifications flag.
 */
void BLE2902::setNotifications(bool flag) {
	uint8_t *pValue = getValue();
	if (flag) pValue[0] |= 1 << 0;
	else pValue[0] &= ~(1 << 0);
}