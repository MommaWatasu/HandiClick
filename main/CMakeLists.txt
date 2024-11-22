idf_component_register(SRCS "main.cpp" "./libraries/BLE_Mouse/BleConnectionStatus.cpp" "./libraries/BLE_Mouse/BleMouse.cpp" "./libraries/BLE_Mouse/multi_pair.cpp" "./libraries/Madgwick/MadgwickAHRS.cpp" "./libraries/RotaryEncoder/RotaryEncoder.cpp"
                    PRIV_REQUIRES spi_flash nvs_flash esp_timer arduino-esp32 esp_pm
                    INCLUDE_DIRS "./libraries/BLE_Mouse" "./libraries/Madgwick" "./libraries/RotaryEncoder")
