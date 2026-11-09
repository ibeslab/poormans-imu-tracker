#include <Arduino.h>
#include <Wire.h>

// Identical firmware on both ESP8266 D1 Mini boards.
// USB serial identifies which board is upper arm vs forearm on the laptop.
static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint32_t BAUD = 115200;
static constexpr uint32_t SAMPLE_PERIOD_US = 10000; // 100 Hz

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
uint32_t nextSampleUs = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                       // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(14)) != 14) return false;

  ax = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  ay = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  az = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read();               // Temperature
  gx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  gy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  gz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  return true;
}

void calibrateGyro() {
  Serial.println(F("# Keep IMU still: estimating gyro bias (~1.5s)"));
  int64_t sumX = 0, sumY = 0, sumZ = 0;
  int valid = 0;
  for (int i = 0; i < 300; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sumX += gx; sumY += gy; sumZ += gz;
      ++valid;
    }
    delay(5);
  }
  if (valid > 0) {
    gyroBiasX = static_cast<float>(sumX) / valid;
    gyroBiasY = static_cast<float>(sumY) / valid;
    gyroBiasZ = static_cast<float>(sumZ) / valid;
  }
  Serial.println(F("# Ready; CSV: ms,ax,ay,az,gx,gy,gz (g,deg/s)"));
}

void setup() {
  Serial.begin(BAUD);
  delay(300);
  Wire.begin(D2, D1);                    // SDA D2=GPIO4, SCL D1=GPIO5
  Wire.setClock(400000);                 // Fast-mode I2C
  bool ok = true;
  ok &= writeReg(0x6B, 0x01);            // Wake; gyro X as clock
  ok &= writeReg(0x1A, 0x03);            // Digital low-pass (~42-44Hz)
  ok &= writeReg(0x19, 0x09);            // 1kHz / (1+9) = 100Hz
  ok &= writeReg(0x1B, 0x00);            // Gyro +/-250 deg/s
  ok &= writeReg(0x1C, 0x00);            // Accel +/-2g
  if (!ok) {
    Serial.println(F("# ERROR: MPU6050 config failed"));
    while (true) delay(1000);
  }
  delay(100);
  calibrateGyro();
  nextSampleUs = micros() + SAMPLE_PERIOD_US;
}

void loop() {
  uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_PERIOD_US;
  if (static_cast<int32_t>(now - nextSampleUs) > 0) {
    nextSampleUs = now + SAMPLE_PERIOD_US; // Skip samples if behind
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (!readMPU(ax, ay, az, gx, gy, gz)) return;
  constexpr float A_SCALE = 1.0f / 16384.0f;
  constexpr float G_SCALE = 1.0f / 131.0f;
  Serial.printf("%lu,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",
      static_cast<unsigned long>(millis()),
      ax * A_SCALE, ay * A_SCALE, az * A_SCALE,
      (gx - gyroBiasX) * G_SCALE,
      (gy - gyroBiasY) * G_SCALE,
      (gz - gyroBiasZ) * G_SCALE);
}
