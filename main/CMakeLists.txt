idf_component_register(SRCS 
#    "bluetooth_first_try.cpp"
#    "pca_testing.cpp"
    "hexapod_testing.cpp"
    "hexapod.cpp"
    "hexapod_leg.cpp"
    "pca9685_handler.cpp"
#    "bluetooth/utils/BLE2902.cpp" "bluetooth/utils/BLEAddress.cpp" 
#                    "bluetooth/utils/BLEAdvertisedDevice.cpp" "bluetooth/utils/BLEAdvertising.cpp" 
#                    "bluetooth/utils/BLECharacteristic.cpp" "bluetooth/utils/BLECharacteristicMap.cpp"
#                    "bluetooth/utils/BLEClient.cpp" "bluetooth/utils/BLEDescriptor.cpp"
#                    "bluetooth/utils/BLEDescriptorMap.cpp" "bluetooth/utils/BLEDevice.cpp" 
#                    "bluetooth/utils/BLEExceptions.cpp" "bluetooth/utils/BLERemoteCharacteristic.cpp" 
#                    "bluetooth/utils/BLERemoteDescriptor.cpp" "bluetooth/utils/BLERemoteService.cpp" 
#                    "bluetooth/utils/BLEScan.cpp" "bluetooth/utils/BLESecurity.cpp" 
#                    "bluetooth/utils/BLEServer.cpp" "bluetooth/utils/BLEService.cpp"
#                    "bluetooth/utils/BLEServiceMap.cpp" "bluetooth/utils/BLEUtils.cpp" 
#                    "bluetooth/utils/BLEUUID.cpp" "bluetooth/utils/BLEValue.cpp" 
#                    "bluetooth/utils/FreeRTOSInterface.cpp" "bluetooth/utils/GeneralUtils.cpp"
#                    "bluetooth/bluetooth/Bluetooth.cpp"
    
#    INCLUDE_DIRS 
#    "bluetooth/utils/include" 
#    "bluetooth/bluetooth/include"
#    ".")
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

