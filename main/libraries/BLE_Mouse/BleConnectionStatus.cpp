#include "BleConnectionStatus.h"
#include <BLEAddress.h>
#include <BLEDevice.h>

#define MAX_BONDED_DEVICES 2 // Maximum number of bonded devices
#define LOG_TAG "BLEConnectionStatus"

int32_t num_bonded_dev = 0;
int device_num = 0;
esp_ble_bond_dev_t bonded_devices[MAX_BONDED_DEVICES];
esp_bd_addr_t connected_device_addr;
BondedDevice inactive_bonded_device = EmptyBondedDevice;

BleConnectionStatus::BleConnectionStatus(void) {
}

void BleConnectionStatus::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param)
{
  this->connected = true;
  BLE2902* desc = (BLE2902*)this->inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(true);
  
  memcpy(connected_device_addr, param->connect.remote_bda, sizeof(esp_bd_addr_t));
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
          param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
  if (num_bonded_dev == 0) {
    ESP_LOGI(LOG_TAG, "Bonding device with MAC address: %18s", macStr);
    pServer->getAdvertising()->stop();

    ESP_LOGI(LOG_TAG, "Updating bonded devices");
    int list_size = 2;
    esp_ble_get_bond_device_list(&list_size, bonded_devices);
  } else if (num_bonded_dev == 1) {
    if (device_num == 1 && memcmp(param->connect.remote_bda, bonded_devices[0].bd_addr, sizeof(param->connect.remote_bda)) != 0) {
      ESP_LOGI(LOG_TAG, "Bonding device with MAC address: %18s", macStr);
      pServer->getAdvertising()->stop();

      ESP_LOGI(LOG_TAG, "Updating bonded devices");
      int list_size = 2;
      esp_ble_get_bond_device_list(&list_size, bonded_devices);

      esp_ble_gap_clear_whitelist();
      BLEAddress address = BLEAddress(bonded_devices[device_num].bd_addr);
      BLEDevice::whiteListAdd(address);
      pServer->getAdvertising()->setScanFilter(true, true);
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
  pServer->startAdvertising();
  ESP_LOGI(LOG_TAG, "Disconnected");
}
