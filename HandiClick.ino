#include <array>
#include <BleMouse.h>
#include <MadgwickAHRS.h>
#include <Ticker.h>
#include <Wire.h>

// I2C address of BMX055 acceleration sensor
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Open)
// I2C address of BMX055 gyro sensor
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Open)
// I2C address of BMX055 magnetic sensor
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Open)

//
#define GPIO_PIN1 2

struct BMX055 {
  // unite: m/s^2
  float accel[3];
  float gyro[3];
  float mag[3];

  BMX055() {
    accel[0], accel[1], accel[2] = 0.00, 0.00, 0.00;
    gyro[0], gyro[1], gyro[2] = 0.00, 0.00, 0.00;
    gyro[0], gyro[1], gyro[2] = 0.00, 0.00, 0.00;
  }

  void init() {
    //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03);   // Range = +/- 2g
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x10);  // Select PMU_BW register
    Wire.write(0x0F);  // Bandwidth = 1000 Hz
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Accl);
    Wire.write(0x11);  // Select PMU_LPW register
    Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x0F);  // Select Range register
    Wire.write(0x04);  // Full scale = +/- 125 degree/s
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x10);  // Select Bandwidth register
    Wire.write(0x07);  // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Gyro);
    Wire.write(0x11);  // Select LPM1 register
    Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x83);  // Soft reset
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x01);  // Soft reset
    Wire.endTransmission();
    delay(100);
  //------------------------------------------------------------//
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x4C);  // Select Mag register
    Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
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
    delay(100);
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
  int zero_count[3];
  int tilt_count;
  float old_accel[3];
  float avg_accel[3];
  float default_accel[3];
  float vel[3];
  float dpos[2];

  Motion2D() {
    zero_count[0], zero_count[1], zero_count[2] = 0, 0, 0;
    tilt_count = 0;
    old_accel[0], old_accel[1], old_accel[2] = 0.0, 0.0, 0.0;
    avg_accel[0], avg_accel[1], avg_accel[2] = 0.0, 0.0, 0.0;
    vel[0], vel[1], vel[2] = 0.0, 0.0, 0.0;
    dpos[0], dpos[1] = 0.0, 0.0;
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
    return std::round(vel[0]);
  }

  signed char dy() {
    return -std::round(dpos[1]);
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
    update_madgewick(bmx.accel, bmx.gyro, bmx.mag);
    default_z_accel = z_accel;
    avg_z_accel = bmx.accel[2];
    default_posture[0], default_posture[1], default_posture[2] = pitch, roll, yaw;
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
      return (x - 10) / 10;
    } else {
      return (x + 10) / 10;
    }
  }

  signed char dx() {
    signed char dx = std::round(convert_range(pitch));
    if (dx > 6) {
      return 6;
    } else if (dx < -6) {
      return -6;
    } else {
      return dx;
    }
  }

  signed char dy() {
    signed char dy = std::round(convert_range(roll));
    if (dy > 6) {
      return 6;
    } else if (dy < -6) {
      return -6;
    } else {
      return dy;
    }
  }
};

Motion2D motion2d;
Motion3D motion3d;
BleMouse bleMouse("HandiClick", "GateHorse", 100);
Ticker MouseTicker;

// now calibration isn't executed. Add processing into the function which is called when the button is pushed.
void mouse2d() {
  signed char dx, dy;
  bool mode = true;

  bmx.update_accel();
  motion2d.update_velocity(bmx.accel);

  mode = motion2d.check_mode();
  if (!mode) {
    MouseTicker.detach();
    delay(10);
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
  motion3d.update_madgewick(bmx.accel, bmx.gyro, bmx.mag);

  mode = motion3d.check_mode();
  if (!mode) {
    MouseTicker.detach();
    delay(10);
    MouseTicker.attach_ms(1, mouse2d);
    return;
  }

  if (bleMouse.isConnected()) {
    dx = motion3d.dx();
    dy = motion3d.dy();
    if (dx != 0 || dy != 0) {
      bleMouse.move(dx*5, dy*5, 0);
    }
  }
}

void left_click() {
  if (bleMouse.isConnected()) {
    bleMouse.click();
  }
}

void setup()
{
  // Initialize Wire(ESP32-I2C)
  Wire.begin();
  // Serial communication using 115200bps
  Serial.begin(115200);

  // Initialize BLE Mouse
  bleMouse.begin();

  // Initialize BMX055
  bmx.init();
  delay(300);

  motion2d.init();
  motion3d.init(motion2d.default_accel[2]);
  MouseTicker.attach_ms(1, mouse2d);
  // set the interruption handler for right click.
  attachInterrupt(GPIO_PIN1, left_click, FALLING);
}

void loop()
{
}