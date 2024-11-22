#ifndef ESP32_BLE_CONNECTION_STATUS_H
#define ESP32_BLE_CONNECTION_STATUS_H
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#define MAX_BONDED_DEVICES 2 // Maximum number of bonded devices

#include <BLEServer.h>
#include "BLE2902.h"
#include "BLECharacteristic.h"
#include "multi_pair.h"

extern int32_t num_bonded_dev;
extern int device_num;
extern esp_ble_bond_dev_t bonded_devices[MAX_BONDED_DEVICES];
extern esp_bd_addr_t connected_device_addr;
extern BondedDevice inactive_bonded_device;

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
