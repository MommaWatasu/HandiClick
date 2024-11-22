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

TimerHandle_t timer_left = NULL;
TimerHandle_t timer_right = NULL;
TimerHandle_t timer_wheel = NULL;
TimerHandle_t timer_switch = NULL;
// time in ms to trigger the watchdog
const int wdtTimeout = 10;
const int dt = 2; // time in ms to update 2d mouse

int sleep_mode_count = 0;

// structure to store the events(move, press, wheel) of the mouse
struct MouseEvent {
  enum class Type {
    NONE,
    MOVE,
    PRESS,
    RELEASE,
    SWITCH
  };

  Type type;
  union {
    struct {
      signed char dx;
      signed char dy;
      signed char wheel;
    } move;
    struct {
      uint8_t button;
    } press;
    struct {
      uint8_t button;
    } release;
    struct {
      uint8_t device;
    } switch_device;
  };

  MouseEvent() : type(Type::NONE) {}
  MouseEvent(Type t) : type(t) {}
};

// FIFO Queue for Events
struct EventQueue {
  MouseEvent event_queue[10];
  int head;
  int tail;

  EventQueue() : head(0), tail(0) {}

  bool push(const MouseEvent &event) {
    if (full()) {
      return false;
    }
    event_queue[tail] = event;
    tail = (tail + 1) % 10;
    return true;
  }

  MouseEvent pop() {
    if (empty()) {
      return MouseEvent(MouseEvent::Type::NONE);
    }
    MouseEvent event = event_queue[head];
    head = (head + 1) % 10;
    return event;
  }

  bool empty() {
    return head == tail;
  }

  bool full() {
    return (tail + 1) % 10 == head;
  }
};
EventQueue event_queue;

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
    Wire.write(0x0d);  // Bandwidth = 250 Hz interval = 2ms
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

  bool check_wakeup(float accel[3]) {
    if (abs(accel[0] - default_accel[0]) > 0.08 || abs(accel[1] - default_accel[1]) > 0.08 || abs(accel[2] - default_accel[2]) > 0.08) {
      return true;
    }
    return false;
  }

  signed char dx() {
    return -std::round(vel[0]) * dt;
  }

  signed char dy() {
    return std::round(dpos[1]) * dt;
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
void check_wakeup();

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
    if (dx == 0 && dy == 0) {
      sleep_mode_count += 1;
      if (sleep_mode_count > 100) {
        sleep_mode_count = 0;
        MouseTicker.detach();
        MouseTicker.attach_ms(50, check_wakeup);
      }
    } else {
      sleep_mode_count = 0;
      // define event
      MouseEvent event = MouseEvent(MouseEvent::Type::MOVE);
      event.move.dx = dx;
      event.move.dy = dy;
      event.move.wheel = 0;
      // enqueue the event
      event_queue.push(event);
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
    motion2d.init();
    MouseTicker.attach_ms(dt, mouse2d);
    return;
  }

  if (bleMouse.isConnected()) {
    dx = motion3d.dx();
    dy = motion3d.dy();
    if (dx != 0 || dy != 0) {
      // define event
      MouseEvent event = MouseEvent(MouseEvent::Type::MOVE);
      event.move.dx = dx;
      event.move.dy = dy;
      event.move.wheel = 0;
      // enqueue the event
      event_queue.push(event);
    }
  }
}

void check_wakeup() {
  bmx.update_accel();
  motion2d.update_velocity(bmx.accel.data());

  if (motion2d.check_wakeup(bmx.accel.data())) {
    MouseTicker.detach();
    MouseTicker.attach_ms(dt, mouse2d);
  }
}

// set the interruption handler for left click
void left_click() {
  if (bleMouse.isConnected()) {
    bleMouse.click(MOUSE_LEFT);
  }
  detachInterrupt(GPIO_PIN_LEFT);
  xTimerStart(timer_left, 0);
}

// set the interruption handler for right click
void right_click() {
  if (bleMouse.isConnected()) {
    bleMouse.click(MOUSE_RIGHT);
  }
  detachInterrupt(GPIO_PIN_RIGHT);
  xTimerStart(timer_right, 0);
}

// set the interruption handler for left drag
void left_click_rm() {
  if (bleMouse.isConnected()) {
    // define event
    MouseEvent event = MouseEvent(MouseEvent::Type::RELEASE);
    event.release.button = MOUSE_LEFT;
    // enqueue the event
    event_queue.push(event);
  }
  detachInterrupt(GPIO_PIN_LEFT);
  xTimerStart(timer_left, 0);
}

// set the interruption handler for right drag
void right_click_rm() {
  if (bleMouse.isConnected()) {
    // define event
    MouseEvent event = MouseEvent(MouseEvent::Type::RELEASE);
    event.release.button = MOUSE_RIGHT;
    // enqueue the event
    event_queue.push(event);
  }
  detachInterrupt(GPIO_PIN_RIGHT);
  xTimerStart(timer_right, 0);
}

void enable_left_click(TimerHandle_t _) {
  if (digitalRead(GPIO_PIN_LEFT) == HIGH) {
    attachInterrupt(GPIO_PIN_LEFT, left_click_rm, FALLING);
    if (bleMouse.isConnected()) {
      // define event
      MouseEvent event = MouseEvent(MouseEvent::Type::PRESS);
      event.press.button = MOUSE_LEFT;
      // enqueue the event
      event_queue.push(event);
    }
  } else {
    attachInterrupt(GPIO_PIN_LEFT, left_click, RISING);
  }
  xTimerStop(timer_left, 0);
}

void enable_right_click(TimerHandle_t _) {
  if (digitalRead(GPIO_PIN_RIGHT) == HIGH) {
    attachInterrupt(GPIO_PIN_RIGHT, right_click_rm, FALLING);
    if (bleMouse.isConnected()) {
      // define event
      MouseEvent event = MouseEvent(MouseEvent::Type::PRESS);
      event.press.button = MOUSE_RIGHT;
      // enqueue the event
      event_queue.push(event);
    }
  } else {
    attachInterrupt(GPIO_PIN_RIGHT, right_click, RISING);
  }
  xTimerStop(timer_right, 0);
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
      // define event
      MouseEvent event = MouseEvent(MouseEvent::Type::MOVE);
      event.move.wheel = -char(encoder.getDirection()) * 2;
      // enqueue the event
      event_queue.push(event);
    }
    pos = new_pos;
  }
  detachInterrupt(GPIO_PIN_ROTARY_A);
  detachInterrupt(GPIO_PIN_ROTARY_B);
  xTimerStart(timer_wheel, 0);
}

void enable_wheel(TimerHandle_t _) {
  attachInterrupt(GPIO_PIN_ROTARY_A, update_wheel, CHANGE);
  attachInterrupt(GPIO_PIN_ROTARY_B, update_wheel, CHANGE);
  xTimerStop(timer_wheel, 0);
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

  // define event
  MouseEvent event = MouseEvent(MouseEvent::Type::SWITCH);
  event.switch_device.device = device_num;
  // enqueue the event
  event_queue.push(event);

  detachInterrupt(GPIO_PIN_DEVICE_SWITCH);
  xTimerStart(timer_switch, 0);
}

void enable_switch(TimerHandle_t _) {
  xTimerStop(timer_switch, 0);
}

// initialize the device switch
void initialize_switch() {
  // Load bonded devices from NVS
  load_bonded_devices();
  // Load inactive bond device from NVS
  inactive_bonded_device.load();
  pinMode(GPIO_PIN_DEVICE_SWITCH, INPUT_PULLUP);
  if (digitalRead(GPIO_PIN_DEVICE_SWITCH) == HIGH) {
    device_num = 1;
    ESP_LOGI(LOG_TAG, "Device 1");
  } else {
    device_num = 0;
    ESP_LOGI(LOG_TAG, "Device 0");
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

  // Create timers to prevent chattering
  timer_left = xTimerCreate("TimerLeft", pdMS_TO_TICKS(wdtTimeout), pdFALSE, NULL, enable_left_click);
  timer_right = xTimerCreate("TimerRight", pdMS_TO_TICKS(wdtTimeout), pdFALSE, NULL, enable_right_click);
  timer_wheel = xTimerCreate("TimerWheel", pdMS_TO_TICKS(2), pdFALSE, NULL, enable_wheel);
  timer_switch = xTimerCreate("TimerSwitch", pdMS_TO_TICKS(wdtTimeout), pdFALSE, NULL, enable_switch);

  // Initialize BMX055
  bmx.init();
  delay(300); // 300ms

  motion2d.init();
  motion3d.init(motion2d.default_accel[2]);
  MouseTicker.attach_ms(dt, mouse2d);

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
  while (1) {
    if (event_queue.empty()) {
      delay(dt);
      continue;
    } else {
      MouseEvent event = event_queue.pop();
      switch (event.type) {
        case MouseEvent::Type::MOVE:
          if (bleMouse.isConnected()) {
            bleMouse.move(event.move.dx, event.move.dy, event.move.wheel);
          }
          break;
        case MouseEvent::Type::PRESS:
          if (bleMouse.isConnected()) {
            bleMouse.press(event.press.button);
          }
          break;
        case MouseEvent::Type::RELEASE:
          if (bleMouse.isConnected()) {
            bleMouse.release(event.release.button);
          }
          break;
        case MouseEvent::Type::SWITCH:
          if (event.switch_device.device == 0) {
            ESP_LOGI(LOG_TAG, "Device 0");
          } else {
            ESP_LOGI(LOG_TAG, "Device 1");
          }
          if (memcmp(connected_device_addr, bonded_devices[event.switch_device.device].bd_addr, sizeof(connected_device_addr)) != 0) {
            esp_ble_gap_disconnect(connected_device_addr);
          }
          break;
        default:
          break;
      }
    }
    // check free heap memory and if it's nearly full, wait for a while
    while (ESP.getFreeHeap() < 1000) {
      delay(2);
    }
  }
}