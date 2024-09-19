#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

TwoWire::TwoWire() {
    _i2c_num = I2C_MASTER_NUM;
    _sda_pin = I2C_MASTER_SDA_IO;
    _scl_pin = I2C_MASTER_SCL_IO;
    _freq = I2C_MASTER_FREQ_HZ;
    _index = 0;
    _length = 0;
}

void TwoWire::begin() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = _sda_pin,
        .scl_io_num = _scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = _freq
        }
    };

    i2c_param_config(_i2c_num, &conf);
    i2c_driver_install(_i2c_num, conf.mode, 0, 0, 0);
}

void TwoWire::beginTransmission(uint8_t addr) {
    _addr = addr;
    _index = 0;
    _length = 0;
}

void TwoWire::write(uint8_t data) {
    if (_index < sizeof(_buffer)) {
        _buffer[_index++] = data;
        _length++;
    }
}

void TwoWire::write(uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        write(data[i]);
    }
}

void TwoWire::endTransmission() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, _buffer, _length, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    _index = 0;
    _length = 0;
}

void TwoWire::requestFrom(uint8_t addr, size_t length) {
    if (length > sizeof(_buffer)) {
        length = sizeof(_buffer);
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, _buffer, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, _buffer + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        _index = 0;
        _length = length;
    }
}

size_t TwoWire::available() {
    return _length - _index;
}

uint8_t TwoWire::read() {
    if (_index < _length) {
        return _buffer[_index++];
    }
    return -1;
}
/*
  Wire.h - esp32 library for I2C communication
TwoWire Wire(0);  // I2C_NUM_0 を使用するインスタンスを作成

// 使用例
void app_main() {
    Wire.begin(21, 22, 100000);  // SDA: GPIO21, SCL: GPIO22, 周波数: 100kHz

    // デバイスへの書き込み
    Wire.beginTransmission(0x68);  // デバイスアドレス 0x68
    Wire.write(0x6B);  // レジスタアドレス
    Wire.write(0x00);  // データ
    Wire.endTransmission(true);

    // デバイスからの読み取り
    Wire.beginTransmission(0x68);
    Wire.write(0x75);  // レジスタアドレス
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1, true);

    if (Wire.available()) {
        uint8_t data = Wire.read();
        printf("Read data: 0x%02X\n", data);
    }
}
*/