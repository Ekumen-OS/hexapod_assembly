#pragma once

#include "sdkconfig.h"
#include <string>
#include "BLEUUID.hpp"
#include "BLECharacteristic.hpp"
#include <esp_gatts_api.h>
#include "FreeRTOSInterface.hpp"

class BLEService;
class BLECharacteristic;
class BLEDescriptorCallbacks;

/**
 * @brief A model of a %BLE descriptor.
 */
class BLEDescriptor {
public:
	BLEDescriptor(const char* uuid, uint16_t max_len = 100);
	BLEDescriptor(BLEUUID uuid, uint16_t max_len = 100);
	virtual ~BLEDescriptor();

	uint16_t getHandle() const;
	size_t   getLength() const;
	BLEUUID  getUUID() const;
	uint8_t* getValue() const;
	void handleGATTServerEvent(
			esp_gatts_cb_event_t      event,
			esp_gatt_if_t             gatts_if,
			esp_ble_gatts_cb_param_t* param);

	void setAccessPermissions(esp_gatt_perm_t perm);
	void setCallbacks(BLEDescriptorCallbacks* pCallbacks);
	void setValue(uint8_t* data, size_t size);
	void setValue(std::string value);

	std::string toString();

private:
	friend class BLEDescriptorMap;
	friend class BLECharacteristic;
	BLEUUID                 m_bleUUID;
	uint16_t                m_handle;
	BLEDescriptorCallbacks* m_pCallback;
	BLECharacteristic*      m_pCharacteristic;
	esp_gatt_perm_t				  m_permissions = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
	FreeRTOSInterface::Semaphore     m_semaphoreCreateEvt = FreeRTOSInterface::Semaphore("CreateEvt");
	esp_attr_value_t        m_value;

	void executeCreate(BLECharacteristic* pCharacteristic);
	void setHandle(uint16_t handle);
}; // BLEDescriptor


/**
 * @brief Callbacks that can be associated with a %BLE descriptors to inform of events.
 *
 * When a server application creates a %BLE descriptor, we may wish to be informed when there is either
 * a read or write request to the descriptors value.  An application can register a
 * sub-classed instance of this class and will be notified when such an event happens.
 */
class BLEDescriptorCallbacks {
public:
	virtual ~BLEDescriptorCallbacks();
	virtual void onRead(BLEDescriptor* pDescriptor);
	virtual void onWrite(BLEDescriptor* pDescriptor);
};
