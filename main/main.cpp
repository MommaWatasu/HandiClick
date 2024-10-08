#include <array>
#include <Arduino.h>
#include <BleMouse.h>
#include <MadgwickAHRS.h>
#include <RotaryEncoder.h>
#include <Ticker.h>
#include <Wire.h>
#include "esp_log.h"
#include "esp_pm.h"

// I2C address of BMX055 acceleration sensor
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Open)
// I2C address of BMX055 gyro sensor
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Open)
// I2C address of BMX055 magnetic sensor
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Open)

#define LOG_TAG "HandiClick"

// Left Click
#define GPIO_PIN_LEFT 32
// Right Click
#define GPIO_PIN_RIGHT 19
// Rotary Encoder 1
#define GPIO_PIN_ROTARY_A 5
// Rotary Encoder 2
#define GPIO_PIN_ROTARY_B 18
// BLE Device Switch
#define GPIO_PIN_DEVICE_SWITCH 13

#define MAX_BONDED_DEVICES 2 // Maximum number of bonded devices

hw_timer_t *timer_left = NULL;
hw_timer_t *timer_right = NULL;
hw_timer_t *timer_wheel = NULL;
hw_timer_t *timer_switch = NULL;
// time in ms to trigger the watchdog
const int wdtTimeout = 10;

struct BMX055 {
  // unite: m/s^2
  std::array<float, 3> accel;
  std::array<float, 3> gyro;
  std::array<float, 3> mag;

  BMX055() {
    accel.fill(0);
    gyro.fill(0);
    mag.fill(0);
  }

  void init() {
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03);   // Range = +/- 2g
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x10);  // Select PMU_BW register
    Wire.write(0x0F);  // Bandwidth = 1000 Hz
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x11);  // Select PMU_LPW register
    Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x0F);  // Select Range register
    Wire.write(0x04);  // Full scale = +/- 125 degree/s
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x10);  // Select Bandwidth register
    Wire.write(0x07);  // ODR = 100 Hz
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x11);  // Select LPM1 register
    Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x83);  // Soft reset
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x01);  // Soft reset
    Wire.endTransmission();
    delay(100); // 100ms
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4C);  // Select Mag register
    Wire.write(0x07);  // Normal Mode, ODR = 30 Hz
    Wire.endTransmission();
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4E);  // Select Mag register
    Wire.write(0x84);  // X, Y, Z-Axis enabled
    Wire.endTransmission();
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x51);  // Select Mag register
    Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x52);  // Select Mag register
    Wire.write(16);  // No. of Repetitions for Z-Axis = 15
    Wire.endTransmission();
  //------------------------------------------------------------//
    delay(100); // 100ms
  }

  void update_accel() {
    unsigned int data[6];
    for (int i = 0; i < 6; i++) {
      Wire.beginTransmission(Addr_Accl);
      Wire.write((2 + i));// Select data register
      Wire.endTransmission();
      Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
      // Read 6 bytes of data
      // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
      if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    accel[0] = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (accel[0] > 2047)  accel[0] -= 4096;
    accel[1] = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (accel[1] > 2047)  accel[1] -= 4096;
    accel[2] = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (accel[2] > 2047)  accel[2] -= 4096;
    accel[0] = accel[0] * 0.0098; // range = +/-2g
    accel[1] = accel[1] * 0.0098; // range = +/-2g
    accel[2] = accel[2] * 0.0098; // range = +/-2g
  }

  void update_gyro() {
    unsigned int data[6];
    for (int i = 0; i < 6; i++) {
      Wire.beginTransmission(Addr_Gyro);
      Wire.write((2 + i));    // Select data register
      Wire.endTransmission();
      Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
      // Read 6 bytes of data
      // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
      if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data
    gyro[0] = (data[1] * 256) + data[0];
    if (gyro[0] > 32767)  gyro[0] -= 65536;
    gyro[1] = (data[3] * 256) + data[2];
    if (gyro[1] > 32767)  gyro[1] -= 65536;
    gyro[2] = (data[5] * 256) + data[4];
    if (gyro[2] > 32767)  gyro[2] -= 65536;

    gyro[0] = gyro[0] * 0.0038; //  Full scale = +/- 125 degree/s
    gyro[1] = gyro[1] * 0.0038; //  Full scale = +/- 125 degree/s
    gyro[2] = gyro[2] * 0.0038; //  Full scale = +/- 125 degree/s
  }

  void update_mag() {
    unsigned int data[8];
    for (int i = 0; i < 8; i++) {
      Wire.beginTransmission(Addr_Mag);
      Wire.write((0x42 + i));    // Select data register
      Wire.endTransmission();
      Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
      // Read 6 bytes of data
      // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
      if (Wire.available() == 1)
        data[i] = Wire.read();
    }
  // Convert the data
    mag[0] = ((data[1] <<5) | (data[0]>>3));
    if (mag[0] > 4095)  mag[0] -= 8192;
    mag[1] = ((data[3] <<5) | (data[2]>>3));
    if (mag[1] > 4095)  mag[1] -= 8192;
    mag[2] = ((data[5] <<7) | (data[4]>>1));
    if (mag[2] > 16383)  mag[2] -= 32768;
  }

  void update() {
    update_accel();
    update_gyro();
    update_mag();
  }

  std::array<float, 3> calibration() {
    // number of sample
    int n = 10;
    std::array<float, 3> sum = {0.0, 0.0, 0.0};
    for (int i = 0; i < n; i++) {
      update_accel();
      sum[0] += accel[0];
      sum[1] += accel[1];
      sum[2] += accel[2];
    }
    sum[0] = sum[0] / n;
    sum[1] = sum[1] / n;
    sum[2] = sum[2] / n;
    return sum;
  }
};

// Global variable to access BMX055 sensor;
BMX055 bmx;

struct Motion2D {
  std::array<int, 3> zero_count;
  int tilt_count;
  std::array<float, 3> old_accel;
  std::array<float, 3> avg_accel;
  std::array<float, 3> default_accel;
  std::array<float, 3> vel;
  std::array<float, 2> dpos;

  Motion2D() {
    zero_count.fill(0);
    tilt_count = 0;
    old_accel.fill(0);
    avg_accel.fill(0);
    vel.fill(0);
    dpos.fill(0);
  }

  void init() {
    std::array<float, 3> result = bmx.calibration();
    for(int i = 0; i < 3; i++) {
        default_accel[i] = result[i];
    }
  }

  void update_velocity(float accel[3]) {
    accel[0] = accel[0] - default_accel[0];
    accel[1] = accel[1] - default_accel[1];
    accel[2] = accel[2] - default_accel[2];

    avg_accel[0] = (avg_accel[0] + accel[0]) / 2;
    avg_accel[1] = (avg_accel[1] + accel[1]) / 2;
    avg_accel[2] = (avg_accel[2] + accel[2]) / 2;

    // store the old velocities
    float old_vel[3] = {vel[0], vel[1], vel[2]};

    // update x velocity
    if (abs(avg_accel[0]) <= 0.08) {
      if (zero_count[0] > 5) {
        zero_count[0] = 0;
        vel[0] = 0.0;
      } else {
        zero_count[0] += 1;
      }
    } else {
      zero_count[0] = 0;
      vel[0] += (old_accel[0] + accel[0]) * 0.2;
    }

    // update y velocity
    if (abs(avg_accel[1]) <= 0.08) {
      if (zero_count[1] > 5) {
        zero_count[1] = 0;
        vel[1] = 0.0;
      } else {
        zero_count[1] += 1;
      }
    } else {
      zero_count[1] = 0;
      vel[1] += (old_accel[1] + accel[1]) * 0.2;
    }

    // update z velocity
    if (abs(avg_accel[2]) <= 0.08) {
      if (zero_count[2] > 5) {
        zero_count[2] = 0;
        vel[2] = 0.0;
      } else {
        zero_count[2] += 1;
      }
    } else {
      zero_count[2] = 0;
      vel[2] += (old_accel[2] + accel[2]) * 0.2;
    }

    // store acceleration values
    old_accel[0] = accel[0];
    old_accel[1] = accel[1];
    old_accel[2] = accel[2];

    // updare position
    dpos[0] = (old_vel[0] + vel[0])/2;
    dpos[1] = (old_vel[1] + vel[1])/2;
  }

  bool check_mode() {
    if (abs(avg_accel[2]) > 0.1) {
      tilt_count += 1;
      if (tilt_count > 10) {
        tilt_count = 0;
        return false;
      }
    } else {
      tilt_count = 0;
    }
    return true;
  }

  signed char dx() {
    return -std::round(vel[0]);
  }

  signed char dy() {
    return std::round(dpos[1]);
  }
};

struct Motion3D {
  Madgwick filter;
  float default_z_accel;
  float avg_z_accel;
  int zero_count;
  float pitch;
  float roll;
  float yaw;
  float default_posture[3];

  Motion3D() {
    filter.begin(30);
    avg_z_accel = 0.0;
    zero_count = 0;
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;
  }

  void init(float z_accel) {
    bmx.update();
    update_madgewick(bmx.accel.data(), bmx.gyro.data(), bmx.mag.data());
    default_z_accel = z_accel;
    avg_z_accel = bmx.accel[2];
    default_posture[0] = pitch;
    default_posture[1] = roll;
    default_posture[2] = yaw;
  }

  void update_madgewick(float accel[3], float gyro[3], float mag[3]) {
    avg_z_accel = (avg_z_accel + accel[2]) / 2;

    filter.update(
      gyro[0], gyro[1], gyro[2],
      accel[0], accel[1], accel[2],
      mag[0], mag[1], mag[2]);

    pitch = filter.getPitch();
    roll = filter.getRoll();
    yaw = filter.getYaw();
  }

  bool check_mode() {
    if (abs(avg_z_accel - default_z_accel) <= 0.1) {
      zero_count += 1;
      if (zero_count > 5) {
        zero_count = 0;
        return false;
      }
    } else {
      zero_count = 0;
    }
    return true;
  }

  float convert_range(float x) {
    if (x > -10 && x < 10) {
      return 0.0;
    } else if (x > 0) {
      return x - 10;
    } else {
      return x + 10;
    }
  }

  signed char dx() {
    return -std::round(convert_range(pitch));
  }

  signed char dy() {
    return -std::round(convert_range(roll));
  }
};

Motion2D motion2d;
Motion3D motion3d;
RotaryEncoder encoder(GPIO_PIN_ROTARY_A, GPIO_PIN_ROTARY_B, RotaryEncoder::LatchMode::TWO03);
int pos = 0;
BleMouse bleMouse("HandiClick", "GateHorse", 100);
Ticker MouseTicker;

void mouse3d();

void mouse2d() {
  signed char dx, dy;
  bool mode = true;

  bmx.update_accel();
  motion2d.update_velocity(bmx.accel.data());

  mode = motion2d.check_mode();
  if (!mode) {
    MouseTicker.detach();
    delay(10); // 10ms
    MouseTicker.attach_ms(33, mouse3d);
    return;
  }

  if(bleMouse.isConnected()) {
    dx = motion2d.dx();
    dy = motion2d.dy();
    if (dx != 0 || dy != 0) {
      bleMouse.move(dx, dy, 0);
    }
  }
}

void mouse3d() {
  signed char dx, dy;
  bool mode;

  bmx.update();
  motion3d.update_madgewick(bmx.accel.data(), bmx.gyro.data(), bmx.mag.data());

  mode = motion3d.check_mode();
  if (!mode) {
    MouseTicker.detach();
    delay(10); // 10ms
    MouseTicker.attach_ms(1, mouse2d);
    return;
  }

  if (bleMouse.isConnected()) {
    dx = motion3d.dx();
    dy = motion3d.dy();
    if (dx != 0 || dy != 0) {
      bleMouse.move(dx, dy, 0);
    }
  }
}

void left_click() {
  if (bleMouse.isConnected()) {
    bleMouse.click(MOUSE_LEFT);
  }
  detachInterrupt(GPIO_PIN_LEFT);
  timerWrite(timer_left, 0);
  timerStart(timer_left);
}

void right_click() {
  if (bleMouse.isConnected()) {
    bleMouse.click(MOUSE_RIGHT);
  }
  detachInterrupt(GPIO_PIN_RIGHT);
  timerWrite(timer_right, 0);
  timerStart(timer_right);
}

void left_click_rm() {
  if (bleMouse.isConnected()) {
    bleMouse.release(MOUSE_LEFT);
  }
  detachInterrupt(GPIO_PIN_LEFT);
  timerWrite(timer_left, 0);
  timerStart(timer_left);
}

void right_click_rm() {
  if (bleMouse.isConnected()) {
    bleMouse.release(MOUSE_RIGHT);
  }
  detachInterrupt(GPIO_PIN_RIGHT);
  timerWrite(timer_right, 0);
  timerStart(timer_right);
}

void enable_left_click() {
  if (digitalRead(GPIO_PIN_LEFT) == HIGH) {
    attachInterrupt(GPIO_PIN_LEFT, left_click_rm, FALLING);
    if (bleMouse.isConnected()) {
      bleMouse.press(MOUSE_LEFT);
    }
  } else {
    attachInterrupt(GPIO_PIN_LEFT, left_click, RISING);
  }
  timerStop(timer_left);
}

void enable_right_click() {
  if (digitalRead(GPIO_PIN_RIGHT) == HIGH) {
    attachInterrupt(GPIO_PIN_RIGHT, right_click_rm, FALLING);
    if (bleMouse.isConnected()) {
      bleMouse.press(MOUSE_RIGHT);
    }
  } else {
    attachInterrupt(GPIO_PIN_RIGHT, right_click, RISING);
  }
  timerStop(timer_right);
}

// set the interruption handler for left and right click
void initialize_clicks() {
  pinMode(GPIO_PIN_LEFT, INPUT_PULLUP);
  attachInterrupt(GPIO_PIN_LEFT, left_click, RISING);
  pinMode(GPIO_PIN_RIGHT, INPUT_PULLUP);
  attachInterrupt(GPIO_PIN_RIGHT, right_click, RISING);
}

/*
// Here is the implementation for Wheel
*/
void update_wheel() {
  encoder.tick();

  int new_pos = encoder.getPosition();
  if (pos != new_pos) {
    if (bleMouse.isConnected()) {
      bleMouse.move(0, 0, -char(encoder.getDirection()) * 2);
    }
    pos = new_pos;
  }
  detachInterrupt(GPIO_PIN_ROTARY_A);
  detachInterrupt(GPIO_PIN_ROTARY_B);
  timerWrite(timer_wheel, 0);
  timerStart(timer_wheel);
}

void enable_wheel() {
  attachInterrupt(GPIO_PIN_ROTARY_A, update_wheel, CHANGE);
  attachInterrupt(GPIO_PIN_ROTARY_B, update_wheel, CHANGE);
  timerStop(timer_wheel);
}

void initialize_wheel() {
  attachInterrupt(GPIO_PIN_ROTARY_A, update_wheel, CHANGE);
  attachInterrupt(GPIO_PIN_ROTARY_B, update_wheel, CHANGE);
}

void switch_ble_device() {
  if (digitalRead(GPIO_PIN_DEVICE_SWITCH) == HIGH) {
    device_num = 1;
  } else {
    device_num = 0;
  }
  detachInterrupt(GPIO_PIN_DEVICE_SWITCH);
  timerWrite(timer_switch, 0);
  timerStart(timer_switch);
}

void enable_switch() {
  timerStop(timer_switch);
}

// initialize the device switch
void initialize_switch() {
  pinMode(GPIO_PIN_DEVICE_SWITCH, INPUT_PULLUP);
  // Initialize the device_num to device which device to connect
  if (digitalRead(GPIO_PIN_DEVICE_SWITCH) == HIGH) {
    device_num = 1;
    ESP_LOGI(LOG_TAG, "Device 1");
    esp_ble_gap_update_whitelist(false, bonded_devices[0].bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
    esp_ble_gap_update_whitelist(true, bonded_devices[1].bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
  } else {
    device_num = 0;
    ESP_LOGI(LOG_TAG, "Device 0");
    esp_ble_gap_update_whitelist(false, bonded_devices[1].bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
    esp_ble_gap_update_whitelist(true, bonded_devices[0].bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
  }
  // set the interruption handler for device switch
  attachInterrupt(GPIO_PIN_DEVICE_SWITCH, switch_ble_device, CHANGE);
}

extern "C" void app_main()
{
  // Initialize Arduino
  initArduino();
  // Initialize Wire(ESP32-I2C)
  Wire.begin();
  // Serial communication using 115200bps
  Serial.begin(115200);

  // configure timers to prevent chattering
  timer_left = timerBegin(1000000);
  timer_right = timerBegin(1000000);
  timer_wheel = timerBegin(1000000);
  timer_switch = timerBegin(1000000);
  timerStop(timer_left);
  timerStop(timer_right);
  timerStop(timer_wheel);
  timerStop(timer_switch);
  timerAttachInterrupt(timer_left, &enable_left_click);
  timerAttachInterrupt(timer_right, &enable_right_click);
  timerAttachInterrupt(timer_wheel, &enable_wheel);
  timerAttachInterrupt(timer_switch, &enable_switch);
  timerAlarm(timer_left, wdtTimeout * 1000, false, 0);
  timerAlarm(timer_right, wdtTimeout * 1000, false, 0);
  timerAlarm(timer_wheel, 2 * 1000, false, 0);
  timerAlarm(timer_switch, wdtTimeout * 1000, false, 0);

  // Initialize BMX055
  bmx.init();
  delay(300); // 300ms

  motion2d.init();
  motion3d.init(motion2d.default_accel[2]);
  MouseTicker.attach_ms(1, mouse2d);

  ESP_LOGI(LOG_TAG, "End of the initialization");

  // Initialize the interruption handler for several GPIO pins
  initialize_clicks();
  initialize_wheel();
  initialize_switch();

  // Configure power management to enable automatic light sleep
  esp_pm_config_t pm_config;
  pm_config.max_freq_mhz = 240; // Set the maximum frequency to 80 MHz
  pm_config.min_freq_mhz = 10; // Set the minimum frequency to 10 MHz
  pm_config.light_sleep_enable = true; // Enable light sleep

  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

  // Initialize BLE Mouse
  bleMouse.begin();
}