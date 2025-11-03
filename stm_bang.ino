// STM8S103F3 — Sduino plain-C
// LSM6DSO @ 0x6A: read accel and compute roll; drive PC3/PC4 by angle
// I2C: PB4=SDA, PB5=SCL

#include <stdint.h>
#include <Wire.h>

// ---- Device / registers ----
#define LSM_ADDR      0x6A   // use 0x6B if SDO is HIGH
#define REG_WHO_AM_I  0x0F   // should read 0x6C on LSM6DSO
#define LSM_WHOAMI    0x6C

#define REG_CTRL1_XL  0x10   // accel ODR/scale
#define REG_CTRL2_G   0x11   // gyro  ODR/scale
#define REG_OUTX_L_A  0x28   // accel X low (little-endian, then 5 more bytes)

// ---- Outputs ----
#define PIN_PC3  PC3
#define PIN_PC4  PC4

// ---------- fast atan2 (no <math.h>) ----------
static float fast_atan2f(float y, float x){
  const float PI_2 = 1.5707963f;
  const float PI_4 = 0.7853982f;
  if (x == 0.0f){
    if (y > 0.0f)  return  PI_2;
    if (y < 0.0f)  return -PI_2;
    return 0.0f;
  }
  float ay = (y < 0.0f) ? -y : y;
  float r, ang;
  if (x > 0.0f){
    r = (x - ay) / (x + ay);
    ang = PI_4 - PI_4 * r;
  } else {
    r = (x + ay) / (ay - x);
    ang = (3.0f*PI_4) - PI_4 * r;
  }
  return (y < 0.0f) ? -ang : ang;
}
#define DEG_PER_RAD 57.29578f

// ---------- I2C helpers ----------
static void i2c_write1(uint8_t addr, uint8_t reg, uint8_t val){
  Wire_beginTransmission(addr);
  Wire_write(reg);
  Wire_write(val);
  Wire_endTransmission();          // Sduino: no args
}

static uint8_t i2c_readN(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n){
  Wire_beginTransmission(addr);
  Wire_write(reg);
  if (Wire_endTransmission()!=0) return 0;
  delay(2);
  uint8_t got = Wire_requestFrom(addr, n);
  for(uint8_t i=0;i<got && Wire_available();++i) buf[i]=Wire_read();
  return got;
}

void setup(void){
  // I2C
  Wire_begin();
  delay(50);

  // Enable accel & gyro (ODR=104 Hz, ±2g and ±250 dps)
  i2c_write1(LSM_ADDR, REG_CTRL1_XL, 0x40); // 0b0100_0000 -> 104 Hz, ±2g
  i2c_write1(LSM_ADDR, REG_CTRL2_G,  0x40); // 0b0100_0000 -> 104 Hz, ±250 dps
  delay(20);

  // Outputs
  pinMode(PIN_PC3, OUTPUT);
  pinMode(PIN_PC4, OUTPUT);
  digitalWrite(PIN_PC3, LOW);
  digitalWrite(PIN_PC4, LOW);
}

void loop(void){
  // Read accelerometer (6 bytes, little-endian)
  uint8_t bA[6];
  if (i2c_readN(LSM_ADDR, REG_OUTX_L_A, bA, 6) == 6){
    int16_t ax = (int16_t)((bA[1]<<8) | bA[0]);
    int16_t ay = (int16_t)((bA[3]<<8) | bA[2]);
    int16_t az = (int16_t)((bA[5]<<8) | bA[4]);

    // Roll around X-axis: atan2(Ay, Az), result in degrees
    float roll_deg = fast_atan2f((float)ay, (float)az) * DEG_PER_RAD;

    // ----- Angle → pins -----
    if (roll_deg < 0.0f) {
      // negative angle
      digitalWrite(PIN_PC3, LOW);
      digitalWrite(PIN_PC4, LOW);
    } else if (roll_deg <= 90.0f) {
      // 0..90°
      digitalWrite(PIN_PC3, HIGH);
      digitalWrite(PIN_PC4, LOW);
    } else {
      // >90°
      digitalWrite(PIN_PC3, LOW);
      digitalWrite(PIN_PC4, HIGH);
    }
  }

  delay(50); // ~20 Hz loop
}
