#include "BleConnectionStatus.h"
#include <BLEAddress.h>
#include <BLEDevice.h>

#define LOG_TAG "BLEConnectionStatus"

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
    memcpy(bonded_devices[0], param->connect.remote_bda, sizeof(esp_bd_addr_t));
    num_bonded_dev++;
    save_bonded_devices();
    printf("%d\n", esp_ble_get_bond_device_num());
    pServer->getAdvertising()->stop();

    ESP_LOGI(LOG_TAG, "Updating bonded devices");
  } else if (num_bonded_dev == 1) {
    if (device_num == 1 && memcmp(param->connect.remote_bda, bonded_devices[0], sizeof(param->connect.remote_bda)) != 0) {
      ESP_LOGI(LOG_TAG, "Bonding device with MAC address: %18s", macStr);
      pServer->getAdvertising()->stop();

      ESP_LOGI(LOG_TAG, "Updating bonded devices");
      memcpy(bonded_devices[1], param->connect.remote_bda, sizeof(esp_bd_addr_t));
      num_bonded_dev++;
      save_bonded_devices();
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
