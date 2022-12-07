set(BLUETOOTH_SRCS 
        "bluetooth/bluetooth.cpp"
        "bluetooth/bt_nimble.cpp"
        "bluetooth/gatt_svr.cpp"
        "bluetooth/misc.cpp"
)

idf_component_register(SRCS 
    "hexapod_testing.cpp"
    "hexapod.cpp"
    "hexapod_leg.cpp"
    "pca9685_handler.cpp"
    ${BLUETOOTH_SRCS}
    
#    INCLUDE_DIRS 
#    ".")
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

