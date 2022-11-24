#include "Bluetooth.hpp"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

const std::string Bluetooth::kServiceUUID("f020f474-36c6-4f9f-9fa5-9736ee68a8f9");

const std::string Bluetooth::kCharacteristicUUID = "eb359b0e-fd69-4cfb-ad0d-9b4d4c3f83db";

Bluetooth::Bluetooth(std::string deviceName) {
    deviceName_ = deviceName;
    start();
}

void Bluetooth::start() {
    BLEDevice::init(deviceName_);
    
    BLEServer* server = BLEDevice::createServer();

    BLEService* service = server->createService(BLEUUID(Bluetooth::kServiceUUID));
    initializeCharacteristic(service);

    service->start();

    BLEAdvertising* advertising = server->getAdvertising();
	advertising->start();
}

void Bluetooth::initializeCharacteristic(BLEService* service) {
    characteristic_ = service->createCharacteristic(
                        BLEUUID(Bluetooth::kCharacteristicUUID),
                        BLECharacteristic::PROPERTY_READ);
}

void Bluetooth::send(std::string value) {
    characteristic_->setValue(value);
}