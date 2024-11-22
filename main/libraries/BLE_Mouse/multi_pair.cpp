#include "multi_pair.h"
#include <nvs_flash.h>
#include <nvs.h>

const char* namespace_name = "HandiClick";

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