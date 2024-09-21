#include "BleConnectionStatus.h"

#define MAX_BONDED_DEVICES 2 // Maximum number of bonded devices
#define LOG_TAG "BLEConnectionStatus"

bool devices_loaded = false;
bool device_full = false;
int list_size = 2;
int device_num = 0;
esp_ble_bond_dev_t bonded_devices[MAX_BONDED_DEVICES];

BleConnectionStatus::BleConnectionStatus(void) {
}

void BleConnectionStatus::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param)
{
  this->connected = true;
  BLE2902* desc = (BLE2902*)this->inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(true);
  if (!devices_loaded) {
    ESP_LOGI(LOG_TAG, "%d", esp_ble_get_bond_device_num());
    ESP_LOGI(LOG_TAG, "Loading bonded devices");
    ESP_ERROR_CHECK(esp_ble_get_bond_device_list(&list_size, bonded_devices));
    devices_loaded = true;
  }
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
          param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
  if (!device_full) {
    if (device_num == 1 && memcmp(param->connect.remote_bda, bonded_devices[0].bd_addr, sizeof(param->connect.remote_bda)) != 0) {
      ESP_LOGI(LOG_TAG, "Bonding device with MAC address: %18s", macStr);
      pServer->getAdvertising()->stop();
      if (esp_ble_get_bond_device_num() == 2) {
        device_full = true;
      }
    } else if (device_num==1) {
      ESP_LOGI(LOG_TAG, "Disconnect Bonded device");
      esp_ble_gap_disconnect(param->connect.remote_bda);
    }
  } else {
    ESP_LOGI(LOG_TAG, "Connected to device with MAC address: %18s", macStr);
    pServer->getAdvertising()->stop();
  }
}

void BleConnectionStatus::onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param)
{
  this->connected = false;
  BLE2902* desc = (BLE2902*)this->inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(false);
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
          param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
  if (device_full) {
    if (memcmp(param->connect.remote_bda, bonded_devices[device_num].bd_addr, sizeof(param->connect.remote_bda)) != 0) {
      esp_ble_gap_update_whitelist(false, param->connect.remote_bda, BLE_WL_ADDR_TYPE_PUBLIC);
      esp_ble_gap_update_whitelist(true, bonded_devices[device_num].bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
    } else {
      uint16_t len;
      esp_ble_gap_get_whitelist_size(&len);
      if (len == 0) {
        esp_ble_gap_update_whitelist(true, param->connect.remote_bda, BLE_WL_ADDR_TYPE_PUBLIC);
      }
    }
  }
  ESP_LOGI(LOG_TAG, "Disconnected");
  pServer->startAdvertising();
}
