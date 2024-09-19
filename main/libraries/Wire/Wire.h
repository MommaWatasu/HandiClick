#ifndef WIRE_H
#define WIRE_H

#define I2C_MASTER_SCL_IO           22      // SCLピンのGPIO番号
#define I2C_MASTER_SDA_IO           21      // SDAピンのGPIO番号
#define I2C_MASTER_NUM              0       // I2Cポート番号
#define I2C_MASTER_FREQ_HZ          100000  // I2C周波数

class TwoWire {
  public:
    TwoWire(uint8_t i2c_num, uint8_t sda_pin, uint8_t scl_pin, uint32_t freq);
    TwoWire();
    void begin();
    void beginTransmission(uint8_t addr);
    void write(uint8_t data);
    void write(uint8_t *data, size_t length);
    void endTransmission();
    void requestFrom(uint8_t addr, size_t length);
    size_t available();
    uint8_t read();

  private:
    uint8_t _i2c_num;
    uint8_t _sda_pin;
    uint8_t _scl_pin;
    uint32_t _freq;
    uint8_t _addr;
    uint8_t _buffer[128];
    size_t _index;
    size_t _length;
};

#endif  // WIRE_H