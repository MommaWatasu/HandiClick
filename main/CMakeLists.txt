idf_component_register(SRCS "main.cpp"
                    PRIV_REQUIRES spi_flash esp_timer arduino-esp32 esp_pm
                    INCLUDE_DIRS "./libraries/BLE_Mouse" "./libraries/Madgwick" "./libraries/RotaryEncoder" "./libraries/Ticker" "./libraries/Wire" "./libraries/utils")
