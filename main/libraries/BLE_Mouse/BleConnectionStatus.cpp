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
  pServer->getAdvertising()->stop();
  ESP_LOGI(LOG_TAG, "Connected");
}

void BleConnectionStatus::onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param)
{
  this->connected = false;
  BLE2902* desc = (BLE2902*)this->inputMouse->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(false);
  pServer->startAdvertising();
  ESP_LOGI(LOG_TAG, "Disconnected");
}
