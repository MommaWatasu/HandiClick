#include <BleMouse.h>
#include<Wire.h>
// I2C address of BMX055 acceleration sensor
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Open)
// I2C address of BMX055 gyro sensor
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Open)
// I2C address of BMX055 magnetic sensor
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Open)

struct BMX055 {
  // unite: m/s^2
  float accel[3];
  float accel_zero[3];
  float gyro[3];
  float mag[3];

  BMX055() {
    accel[0], accel[1], accel[2] = 0.00, 0.00, 0.00;
    accel_zero[0], accel_zero[1], accel_zero[2] = 0.00, 0.00, 0.00;
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
    calibration();
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
    accel[0] = accel[0] * 0.0098 - accel_zero[0]; // range = +/-2g
    accel[1] = accel[1] * 0.0098 - accel_zero[1]; // range = +/-2g
    accel[2] = accel[2] * 0.0098 - accel_zero[2]; // range = +/-2g
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

  void calibration() {
    // number of sample
    int n = 10;
    float sum[3];
    for (int i = 0; i < n; i++) {
      update_accel();
      sum[0] += accel[0];
      sum[1] += accel[1];
      sum[2] += accel[2];
    }
    accel_zero[0] = sum[0] / n;
    accel_zero[1] = sum[1] / n;
    accel_zero[2] = sum[2] / n;
  }
};

struct Motion {
  int zero_count[3];
  float old_accel[3];
  float avg_accel[3];
  float vel[3];
  float dpos[2];

  Motion() {
    zero_count[0], zero_count[1], zero_count[2] = 0, 0, 0;
    old_accel[0], old_accel[1], old_accel[2] = 0.0, 0.0, 0.0;
    avg_accel[0], avg_accel[1], avg_accel[2] = 0.0, 0.0, 0.0;
    vel[0], vel[1], vel[2] = 0.0, 0.0, 0.0;
    dpos[0], dpos[1] = 0.0, 0.0;
  }

  void update_velocity(float accel[3]) {
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
  
  signed char dx() {
    if (abs(avg_accel[2]) <= 0.08) {
      return std::round(vel[0]);
    } else {
      return 0;
    }
  }

  signed char dy() {
    if (abs(avg_accel[2]) <= 0.08) {
      return -std::round(dpos[1]);
    } else {
      return 0;
    }
  }
};

BMX055 bmx;
Motion motion;
BleMouse bleMouse("HandiClick", "GateHorse", 100);

float xVel = 0.00;
float yVel = 0.00;

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
}

void loop()
{
  signed char dx, dy;

  //BMX055 read acceleration
  bmx.update_accel();
  motion.update_velocity(bmx.accel);
  if (motion.vel[0] != 0 || motion.vel[1] != 0) {
    Serial.print("AcclX:");
    Serial.print(bmx.accel[0]);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(bmx.accel[1]);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(bmx.accel[2]);
    Serial.print(",");
    Serial.print("VelX:");
    Serial.print(motion.vel[0]);
    Serial.print(",");
    Serial.print("VelY:");
    Serial.print(motion.vel[1]);
    Serial.println("");
  }

  if(bleMouse.isConnected()) {
    dx = motion.dx();
    dy = motion.dy();
    if (dx != 0 || dy != 0) {
      bleMouse.move(dx, dy, 0);
    }
  }

  delay(2);
}