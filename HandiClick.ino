#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>
#include <Wire.h>

struct Motion {
  float accel[3];
  int zero_count[2];
  float vel[3];
  signed int dpos[2];
  signed int zpos;

  Motion() {
    accel[0], accel[1], accel[2] = 0.0, 0.0, 0.0;
    zero_count[0], zero_count[1] = 0, 0;
    vel[0], vel[1], vel[2] = 0.0, 0.0, 0.0;
    dpos[0], dpos[1] = 0, 0;
  }

  void update(float x, float y, float z) {
    // truncate the input if the input value is too small(noise)
    if (-0.05 < x && x < 0.05) {
      x = 0.0;
      if (zero_count[0] < 3) { zero_count[0] += 1; }
      if (zero_count[0] == 3) {
        vel[0] = 0.0;
        dpos[0] = 0;
      }
    } else {
      zero_count[0] = 0;
    }
    if (-0.05 < y && y < 0.05) {
      y = 0.0;
      if (zero_count[1] < 3) { zero_count[1] += 1; }
      if (zero_count[1] == 3) {
        vel[1] = 0.0;
        dpos[1] = 0;
      }
    } else {
      zero_count[1] = 0;
    }
    if (-0.05 < z && z < 0.05) {
      z = 0.0;
    }

    float old_vel[2] = {vel[0], vel[1]};

    // update velocity
    vel[0] += (x+accel[0])*0.4/2;
    vel[1] += (y+accel[1])*0.4/2;

    if (-0.05 < vel[0] && vel[0] < 0.05) {
      vel[0] = 0.0;
    }
    if (-0.05 < vel[1] && vel[1] < 0.05) {
      vel[1] = 0.0;
    }

    // assign new acceleration
    accel[0] = x;
    accel[1] = y;
    accel[2] = z;

    // updare position
    dpos[0] = std::round((old_vel[0] + vel[0])/2);
    dpos[1] = std::round((old_vel[1] + vel[1])/2);
  }
};

Adafruit_MPU6050 mpu;
BleMouse bleMouse("HandiClick", "GateHorse", 100);
float ZERO_X;
float ZERO_Y;
float ZERO_Z;
Motion motion;
bool PLANE = true;

void setup(void) {
  // Try to initialize Serial Port
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Initialize BLE Mouse
  bleMouse.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println();
  delay(100);

  calibration();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  motion.update(a.acceleration.x-ZERO_X, a.acceleration.y-ZERO_Y, a.acceleration.z-ZERO_Z);

  /*
  Serial.print("AccelX:");
  Serial.print(motion.accel[0]);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(motion.accel[1]);
  Serial.print(",");
  Serial.print("dposX:");
  Serial.print(motion.dpos[0]);
  Serial.print(",");
  Serial.print("dposY:");
  Serial.print(motion.dpos[1]);
  Serial.println();
  */

  Serial.print("velX:");
  Serial.print(motion.vel[0]);
  Serial.print(",");
  Serial.print("accelX:");
  Serial.print(motion.accel[0]);
  Serial.println();

  if(bleMouse.isConnected()) {
    bleMouse.move(motion.dpos[0], -motion.dpos[1], 0);
  }

  delay(10);
}

void calibration() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ZERO_X = a.acceleration.x;
  ZERO_Y = a.acceleration.y;
  ZERO_Z = a.acceleration.z;
}
