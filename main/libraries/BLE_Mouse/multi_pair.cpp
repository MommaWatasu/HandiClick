#include "multi_pair.h"
#include "esp_gap_ble_api.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <cstring>

const char* namespace_name = "HandiClick";

int32_t num_bonded_dev = 0;
int device_num = 0;
esp_bd_addr_t bonded_devices[MAX_BONDED_DEVICES] = {};
esp_bd_addr_t connected_device_addr;
BondedDevice inactive_bonded_device = EmptyBondedDevice;

esp_err_t BondedDevice::save() {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open(namespace_name, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Save structure fields
    err = nvs_set_blob(nvs_handle, "inactive_device", this, sizeof(BondedDevice));
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    // Commit the write
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    return err;
}

esp_err_t BondedDevice::load() {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open(namespace_name, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Read structure fields
    size_t required_size = sizeof(BondedDevice);
    err = nvs_get_blob(nvs_handle, "inactive_device", this, &required_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    // Close NVS handle
    nvs_close(nvs_handle);
    return ESP_OK;
}

void BondedDevice::swap() {
    esp_bd_addr_t bd_addr;

    // create new bonded device structure to store the old information
    BondedDevice new_device = EmptyBondedDevice;
    new_device.device_num = (device_num == 0) ? 1 : 0;
    memcpy(bd_addr, bonded_devices[new_device.device_num], sizeof(esp_bd_addr_t));
    BOOLEAN is_sc_supported;
    esp_ble_get_bond_device_info(&bd_addr, &new_device.dev_class, &new_device.link_key, &new_device.key_type, &new_device.pin_length, &is_sc_supported);
    new_device.sc_support = is_sc_supported;
    sc_support = is_sc_supported;

    // save the new device information
    esp_ble_add_bond_device(this->bd_addr, this->dev_class, this->link_key, this->trusted_mask, this->is_trusted, this->key_type, this->io_cap, this->pin_length, this->sc_support);

    // Update the current device parameters with the new device parameters
    memcpy(this->bd_addr, new_device.bd_addr, sizeof(esp_bd_addr_t));
    memcpy(this->dev_class, new_device.dev_class, sizeof(this->dev_class));
    memcpy(this->link_key, new_device.link_key, sizeof(this->link_key));
    this->trusted_mask = new_device.trusted_mask;
    this->is_trusted = new_device.is_trusted;
    this->key_type = new_device.key_type;
    this->io_cap = new_device.io_cap;
    this->pin_length = new_device.pin_length;
    this->sc_support = new_device.sc_support;
}

void save_bonded_dveices() {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open(namespace_name, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }

    // Save number of bonded devices
    err = nvs_set_i32(nvs_handle, "num_bonded_dev", num_bonded_dev);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return;
    }

    // Save bonded devices
    err = nvs_set_blob(nvs_handle, "bonded_devices", bonded_devices, sizeof(esp_bd_addr_t) * MAX_BONDED_DEVICES);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return;
    }

    // Commit the write
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

void load_bonded_devices() {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open(namespace_name, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }

    // Read number of bonded devices
    err = nvs_get_i32(nvs_handle, "num_bonded_dev", &num_bonded_dev);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return;
    }

    // Read bonded devices
    size_t required_size = sizeof(esp_bd_addr_t) * MAX_BONDED_DEVICES;
    err = nvs_get_blob(nvs_handle, "bonded_devices", bonded_devices, &required_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return;
    }

    // Commit the write
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}