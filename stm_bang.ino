// STM8S103F3 — Sduino plain-C
// I2C scan + 1200-baud bit-banged TX on PC3
// I2C: PB4=SDA, PB5=SCL

#include <stdint.h>
#include <Wire.h>

#define TX_PIN  PC3       // software TX out
#define FLAG_PIN PC4      // goes HIGH if any I2C device is found
#define LSM_ADDR      0x6A   // use 0x6B if SDO pin is tied HIGH
#define REG_WHO_AM_I  0x0F   // should read 0x6C on LSM6DSO
#define LSM_WHOAMI    0x6C

#define REG_OUTX_L_G   0x22  // gyro  X LSB (then 12 more bytes to Z_H_G)
#define REG_OUTX_L_A   0x28  // accel X LSB (then 12 more bytes to Z_H_A)
#define REG_CTRL1_XL   0x10  // accel ODR/scale
#define REG_CTRL2_G    0x11  // gyro  ODR/scale

// write 1 byte: reg <- val
static void i2c_write1(uint8_t addr, uint8_t reg, uint8_t val){
  Wire_beginTransmission(addr);
  Wire_write(reg);
  Wire_write(val);
  Wire_endTransmission();              // Sduino: no args
}

// read N bytes starting at reg into buf
static uint8_t i2c_readN(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n){
  Wire_beginTransmission(addr);
  Wire_write(reg);
  if (Wire_endTransmission()!=0) return 0;
  // tiny pause helps on some clones/adapters
  delay(2);
  uint8_t got = Wire_requestFrom(addr, n);
  for(uint8_t i=0;i<got && Wire_available();++i) buf[i]=Wire_read();
  return got;
}

// --- 1200 baud timing: 1/1200 s ≈ 833.33 µs per bit ---
static inline void delay_bit(void) {
  // 104 us * 8 ≈ 832 us — close enough for 1200 baud
  delayMicroseconds(104 * 8);
}

// --- bit-bang TX (8N1) ---
static void softserial_write(uint8_t b) {
  // start bit (LOW)
  digitalWrite(TX_PIN, LOW);
  delay_bit();

  // 8 data bits, LSB first
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(TX_PIN, (b & 1) ? HIGH : LOW);
    delay_bit();
    b >>= 1;
  }

  // stop bit (HIGH)
  digitalWrite(TX_PIN, HIGH);
  delay_bit();

  // small idle gap helps receiver resync
  delay_bit();
}

// --- helpers ---
static void uputs(const char *s){ while(*s) softserial_write((uint8_t)*s++); }
static void uputnl(void){ softserial_write('\r'); softserial_write('\n'); }
static void uputint(int v){
  char buf[8]; int i=0;
  if(v==0){ softserial_write('0'); return; }
  if(v<0){ softserial_write('-'); v=-v; }
  while(v){ buf[i++]='0'+(v%10); v/=10; }
  while(i--) softserial_write((uint8_t)buf[i]);
}
static void uputhex2(uint8_t b){
  const char *h="0123456789ABCDEF";
  softserial_write((uint8_t)h[(b>>4)&0xF]);
  softserial_write((uint8_t)h[b&0xF]);
}
// --- tiny fast math (no <math.h>) ---
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


void setup(void){
  pinMode(TX_PIN, OUTPUT);

  
  // I2C
  Wire_begin();
  delay(50);

  // enable accel (ODR=104 Hz, ±2g), gyro (ODR=104 Hz, ±250 dps)
  i2c_write1(LSM_ADDR, REG_CTRL1_XL, 0x40); // 0b0100_0000 -> ODR_XL=104 Hz, FS_XL=±2g
  i2c_write1(LSM_ADDR, REG_CTRL2_G,  0x40); // 0b0100_0000 -> ODR_G =104 Hz, FS_G =±250 dps
  delay(20);



  uputs("SoftUART on PC3 @1200"); uputnl();

  // Read WHO_AM_I
  uint8_t id = 0xFF;
  uint8_t n  = i2c_readN(LSM_ADDR, REG_WHO_AM_I, &id, 1);

  uputs("WHO_AM_I read "); uputs(n==1 ? "OK: 0x" : "FAIL: 0x"); uputhex2(id); uputnl();

  if(n==1 && id==LSM_WHOAMI){
    uputs("LSM6DSO connected."); uputnl();
  }else{
    uputs("No LSM6DSO at 0x6A (check wiring or try 0x6B)."); uputnl();
  }
}

void loop(void){
  static uint32_t t0=0; 
  uint32_t t = millis();
  if ((t - t0) >= 100) {            // ~10 Hz
    t0 = t;

    uint8_t bA[6];
    if (i2c_readN(LSM_ADDR, 0x28, bA, 6) == 6) { // OUTX_L_A
      int16_t ax = (int16_t)((bA[1]<<8) | bA[0]);
      int16_t ay = (int16_t)((bA[3]<<8) | bA[2]);
      int16_t az = (int16_t)((bA[5]<<8) | bA[4]);

      // roll (rotation around X-axis)
      float roll_deg = fast_atan2f((float)ay, (float)az) * DEG_PER_RAD;

      // print angle directly in degrees
      uputs("ROLL=");
      int degrees = (int)roll_deg;
      // integer part
      uputint(degrees);
      softserial_write('.');
      // two decimals
      int frac = (int)((roll_deg - degrees) * 100);
      if (frac < 0) frac = -frac;
      if (frac < 10) softserial_write('0');
      uputint(frac);
      uputs(" deg");
      uputnl();

    } else {
      uputs("READFAIL"); uputnl();
    }
  }
}
