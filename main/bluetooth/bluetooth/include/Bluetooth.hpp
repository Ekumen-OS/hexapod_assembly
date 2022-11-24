#pragma once

#include "driver/gpio.h"
#include <iostream>
#include "BLEDevice.hpp"
#include "BLEServer.hpp"
#include "BLEUtils.hpp"
#include <map>
#include "measurements.hpp"

class Bluetooth {
public:
    static const std::string kServiceUUID;
    static const std::string kCharacteristicUUID;

    Bluetooth(std::string deviceName);
    void send(std::string value);

private:
    std::string deviceName_;
    BLECharacteristic* characteristic_;

    void initializeCharacteristic(BLEService* service);
    void start();
};