#ifndef ESP32_BLE_CONNECTION_STATUS_H
#define ESP32_BLE_CONNECTION_STATUS_H
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#include <BLEServer.h>
#include "BLE2902.h"
#include "BLECharacteristic.h"

extern int device_num;

class BleConnectionStatus : public BLEServerCallbacks
{
public:
  BleConnectionStatus(void);
  bool connected = false;
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param);
  void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param);
  BLECharacteristic* inputMouse;
};

#endif // CONFIG_BT_ENABLED
#endif // ESP32_BLE_CONNECTION_STATUS_H
