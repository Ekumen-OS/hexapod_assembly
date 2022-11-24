/*
 * BLERemoteCharacteristic.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kolban
 */

#ifndef COMPONENTS_CPP_UTILS_BLEREMOTECHARACTERISTIC_H_
#define COMPONENTS_CPP_UTILS_BLEREMOTECHARACTERISTIC_H_
#include "sdkconfig.h"

#include <string>

#include <esp_gattc_api.h>

#include "BLERemoteService.hpp"
#include "BLERemoteDescriptor.hpp"
#include "BLEUUID.hpp"
#include "FreeRTOSInterface.hpp"

class BLERemoteService;
class BLERemoteDescriptor;
typedef void (*notify_callback)(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

/**
 * @brief A model of a remote %BLE characteristic.
 */
class BLERemoteCharacteristic {
public:
	~BLERemoteCharacteristic();

	// Public member functions
	bool        canBroadcast();
	bool        canIndicate();
	bool        canNotify();
	bool        canRead();
	bool        canWrite();
	bool        canWriteNoResponse();
	BLERemoteDescriptor* getDescriptor(BLEUUID uuid);
	std::map<std::string, BLERemoteDescriptor*>* getDescriptors();
	uint16_t    getHandle();
	BLEUUID     getUUID();
	std::string readValue();
	uint8_t     readUInt8();
	uint16_t    readUInt16();
	uint32_t    readUInt32();
	void        registerForNotify(notify_callback _callback, bool notifications = true);
	bool        writeValue(uint8_t* data, size_t length, bool response = false);
	bool        writeValue(std::string newValue, bool response = false);
	bool        writeValue(uint8_t newValue, bool response = false);
	std::string toString();
	uint8_t*	readRawData();
	BLERemoteService* getRemoteService();

private:
	BLERemoteCharacteristic(uint16_t handle, BLEUUID uuid, esp_gatt_char_prop_t charProp, BLERemoteService* pRemoteService);
	friend class BLEClient;
	friend class BLERemoteService;
	friend class BLERemoteDescriptor;

	// Private member functions
	void gattClientEventHandler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* evtParam);

	void              removeDescriptors();
	void              retrieveDescriptors();

	// Private properties
	BLEUUID              m_uuid;
	esp_gatt_char_prop_t m_charProp;
	uint16_t             m_handle;
	BLERemoteService*    m_pRemoteService;
	FreeRTOSInterface::Semaphore  m_semaphoreReadCharEvt      = FreeRTOSInterface::Semaphore("ReadCharEvt");
	FreeRTOSInterface::Semaphore  m_semaphoreRegForNotifyEvt  = FreeRTOSInterface::Semaphore("RegForNotifyEvt");
	FreeRTOSInterface::Semaphore  m_semaphoreWriteCharEvt     = FreeRTOSInterface::Semaphore("WriteCharEvt");
	std::string          m_value;
	uint8_t 			 *m_rawData = nullptr;
	notify_callback		 m_notifyCallback = nullptr;

	// We maintain a map of descriptors owned by this characteristic keyed by a string representation of the UUID.
	std::map<std::string, BLERemoteDescriptor*> m_descriptorMap;
}; // BLERemoteCharacteristic
#endif /* COMPONENTS_CPP_UTILS_BLEREMOTECHARACTERISTIC_H_ */
