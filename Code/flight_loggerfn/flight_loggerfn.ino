/*
  flight_logger.ino
  ------------------
  Avionics unit: BMP280 + MPU9250 on I2C, four servos, an ESC/thrust output, a CSV
  data logger on an SD card, status LEDs, and a live WiFi control panel.

  TWO MODES (toggle on the web panel; "Save settings" keeps it across reboots):
    MODE 1  TVC mount + aux fins
              Servo 1 / 2 : TVC mount (pitch about sensor Y / yaw about sensor Z)
              Servo 3 / 4 : aux fins or canards - pitch/yaw assist OR roll control
                            (selectable on the panel)
    MODE 2  four fins under the exhaust (plus layout)
              Servo 1..4  : fins A, B, C, D.  A+C = pitch pair, B+D = yaw pair,
                            all four twisting together = roll. No reaction wheel needed.
  Each mode has its own gains, trims, directions and travel limits on the panel.

  MOUNTING: the MPU9250 stands VERTICALLY in the vehicle.
    -X points to the TOP (nose).  +X points down.
    Y and Z lie in the horizontal plane when the vehicle stands upright.

  Wiring (ESP32 dev board, 3.3V logic throughout):
    I2C (BMP280 + MPU9250): SDA GPIO 19, SCL GPIO 21, VCC 3.3V, GND
    SD card: CS 26, MOSI 13, MISO 27, SCK 14   (FAT32; new file every boot)
    Servo 1 GPIO 4   Servo 2 GPIO 2   Servo 3 GPIO 22   Servo 4 GPIO 23
      (separate 5V supply for loaded servos, common ground)
    THRUST / ESC signal: GPIO 32  (50 Hz servo-style PWM, 1000-2000 us). Signal + GND only.
    LEDs (GPIO -> 220 ohm -> LED anode, cathode -> GND):
      GREEN  status        GPIO 25
      PURPLE SD logging    GPIO 18   (fast blink = logging, slow blink = not logging)
      RED    ARMED         GPIO 33
      RED    spare         GPIO 17   (blinks every half second)
    Log STOP input: GPIO 5  (3.3V = stop logging; internal pull-down)
    ARM input:      GPIO 34 (3.3V = ARMED; input-only, external 10k to GND REQUIRED)

  THRUST SAFETY: the ESC output stays at minimum unless the hardware ARM switch is ON
  and "Thrust enable" is ON on the panel. MANUAL thrust cuts to zero (and disables) if
  the page goes quiet for 2 s. The output is limited by Max thrust and ramps up slowly.

  GREEN status LED: fast blink = starting, slow = gyro calibration (keep still),
  very fast = servo tests, solid = READY, fast blink after start-up = IMU not answering.

  Libraries: Adafruit BMP280 (+ Adafruit Unified Sensor), ESP32Servo. SD/SPI/Preferences
  ship with the ESP32 core.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ---------- WiFi (ESP32 as its own access point) ----------
// Connect your phone to this WiFi network, then browse to 192.168.4.1
const char* AP_SSID = "FlightLogger";
const char* AP_PASSWORD = "flighttest123"; // must be 8+ chars, or set to "" for open network

WebServer server(80);
SPIClass sdSPI(VSPI); // SD card SPI bus, routed to the custom pins below

// ---------- Pin definitions ----------
#define I2C_SDA        19
#define I2C_SCL        21

#define SD_CS_PIN      26
#define SD_MOSI_PIN    13
#define SD_MISO_PIN    27
#define SD_SCK_PIN     14

#define STATUS_LED_PIN 25   // GREEN status LED (via 220 ohm)
#define SD_LED_PIN     18   // SD-logging LED (via 220 ohm). Was LOG_OFF_PIN.
#define ARMED_LED_PIN  33   // red ARMED LED (via 220 ohm)
#define LOG_STOP_PIN    5   // HIGH (3.3V) = stop logging
#define ARM_IN_PIN     34   // HIGH (3.3V) = payload ARMED. Input-only: needs external 10k to GND
#define SPARE_LED_PIN  17   // spare red LED: blinks every half second
#define THRUST_PIN     32   // ESC signal for the BLDC motor (servo-style PWM)

#define SERVO1_PIN      4   // TVC servo 1
#define SERVO2_PIN      2   // TVC servo 2
#define SERVO3_PIN     22   // canard 1
#define SERVO4_PIN     23   // canard 2

// ---------- MPU9250 registers ----------
#define MPU_ADDR       0x68   // 0x69 if AD0 is pulled high
#define MPU_WHO_AM_I   0x75
#define MPU_PWR_MGMT_1 0x6B
#define MPU_CONFIG     0x1A
#define MPU_GYRO_CFG   0x1B
#define MPU_ACCEL_CFG  0x1C
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_INT_PIN_CFG  0x37   // used to enable I2C bypass so we can talk to AK8963 directly

// AK8963 magnetometer (behind the MPU9250, reached via I2C bypass)
#define AK8963_ADDR      0x0C
#define AK8963_WHO_AM_I  0x00
#define AK8963_CNTL1     0x0A
#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03

// ---------- Globals ----------
Adafruit_BMP280 bmp; // I2C
Servo servo1, servo2, servo3, servo4;
Servo* SV[4] = {&servo1, &servo2, &servo3, &servo4};
const int SERVO_PINS[4] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
Servo esc;               // ESC / thrust output

unsigned long lastSampleMs = 0;
// ---------- Runtime settings (edited live from the web panel) ----------
struct ServoCfg { float trim = 0, dir = 1, mn = 0, mx = 180; };   // centre trim (deg), direction, min / max angle
struct Cfg {
  float mode = 0;                                   // 0 = TVC mount + aux fins, 1 = four fins
  ServoCfg sv[2][4];                                // [mode][servo]
  float kp = 1.5, kd = 0.10, maxDef = 40;           // mode 1: TVC mount gains / limit (servo deg)
  float auxFunc = 0;                                // mode 1: aux fins 0 = pitch/yaw assist, 1 = roll control
  float auxKp = 0, auxKd = 0, auxMax = 20;          // aux pitch/yaw assist (0 = parked at centre)
  float auxRollKp = 0, auxRollKd = 0.3;             // aux roll control
  float finKp = 1.0, finKd = 0.08, finMax = 25;     // mode 2: fin pitch/yaw gains / limit
  float finRollKp = 0, finRollKd = 0.3;             // mode 2: fin roll gains
  float tiltTrimY = 0, tiltTrimZ = 0;               // mounting offset (deg)
  float accelAlpha = 0.2, gyroAlpha = 0.3;          // smoothing (1 = none)
  float compAlpha = 0.96, compSpin = 0.98;          // gyro trust for tilt / spin
  float sampleMs = 50;                              // sample + control + log interval
  float dlpf = 3, accelRange = 1, gyroRange = 1, bmpFilter = 4;   // sensor hardware settings
  float thrMethod = 0, hoverThr = 40, tiltComp = 1; // AUTO thrust: 0 = fixed hover %, 1 = altitude hold
  float altKp = 1.0, altKd = 2.0, altTarget = 0;    // altitude hold: % per m, % per m/s, target m
  float thrMin = 0, thrMax = 80, thrRamp = 40;      // AUTO floor (%), max thrust (%), ramp (% per s)
} cfg;
#define SAMPLE_INTERVAL_MS ((unsigned long)cfg.sampleMs)
#define COMP_ALPHA         (cfg.compAlpha)
#define COMP_ALPHA_SPIN    (cfg.compSpin)
#define ACCEL_SMOOTH_ALPHA (cfg.accelAlpha)
#define GYRO_SMOOTH_ALPHA  (cfg.gyroAlpha)

bool manualMode = false;          // manual servo control from the web joystick
float joyX = 0, joyY = 0, joy2X = 0, joy2Y = 0;   // two joysticks, -1..1
unsigned long manualLastMs = 0;
bool webStop = false;             // logging stopped from the web panel
bool prevWantStop = false;
float accelLsb = 8192, gyroLsb = 65.5;   // set by applyMpuConfig()
Preferences prefs;

// Thrust state
bool thrEnable = false;           // web "Thrust enable" switch (also needs the hardware ARM switch)
bool thrAuto = false;             // false = MANUAL slider, true = AUTO
float thrManual = 0;              // manual slider, 0..100 %
float thrActual = 0;              // what the ESC is really being told, 0..100 %
unsigned long thrBeatMs = 0;      // last manual-thrust message (dead-man timer)
float vz = 0, lastAltForVz = 0;   // vertical speed from the barometer
float spinRef = 0;                // roll angle reference, set when ARM switches on
bool prevArmed = false;

float seaLevelHpa = 1013.25; // recalibrate to local pressure for accurate altitude

bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz); // defined below

// ---------- Status LEDs ----------
// Patterns are "on ms, off ms".  (1, 0) = solid ON.
#define LED_STARTUP         100, 100    // GREEN fast blink: booting / initialising
#define LED_GYRO_CAL        500, 500    // GREEN slow blink: keep the board still
#define LED_TVC_TEST         40,  40    // GREEN very fast: TVC servo test
#define LED_CANARD_TEST     200, 200    // GREEN medium: canard servo test
#define LED_READY_SOLID       1,   0    // GREEN solid: ready
#define LED_IMU_FAULT        60,  60    // GREEN fast blink after start-up: IMU not answering
#define SDLED_LOGGING       100, 100    // SD LED fast blink: logging
#define SDLED_NOT_LOGGING   500, 500    // SD LED slow blink: not logging

unsigned int ledOnMs = 100, ledOffMs = 100;        // green LED pattern
unsigned int sdLedOnMs = 500, sdLedOffMs = 500;    // SD LED pattern

void setLedPattern(unsigned int onMs, unsigned int offMs) {
  ledOnMs = onMs;
  ledOffMs = offMs;
}

void setSdLedPattern(unsigned int onMs, unsigned int offMs) {
  sdLedOnMs = onMs;
  sdLedOffMs = offMs;
}

void updateLed() {
  unsigned long phase = millis() % (unsigned long)(ledOnMs + ledOffMs);
  digitalWrite(STATUS_LED_PIN, phase < ledOnMs ? HIGH : LOW);
}

void updateSdLed() {
  unsigned long phase = millis() % (unsigned long)(sdLedOnMs + sdLedOffMs);
  digitalWrite(SD_LED_PIN, phase < sdLedOnMs ? HIGH : LOW);
}

// Like delay(), but keeps the status LEDs blinking while it waits.
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateLed();
    updateSdLed();
    delay(1);
  }
}

// ---------- Payload arming ----------
bool payloadArmed = false;   // true only while the ARM switch is ON (and allowed)
bool armLockout = false;     // switch was already ON at boot: must be cycled OFF then ON

// ---------- IMU health ----------
bool imuOK = false;               // true only while the MPU answers on I2C
int imuFailCount = 0;             // consecutive failed reads
unsigned long lastImuRetryMs = 0;

// ---------- SD logging state ----------
File logFile;
bool sdMounted = false;       // card initialised OK
bool logActive = false;       // file open and recording right now
bool logFileCreated = false;  // a log file has been created this boot
char logFilename[24] = "";
int samplesSinceFlush = 0;
const int FLUSH_EVERY_N_SAMPLES = 10; // flush ~every 0.5 s: caps data lost on a power cut

bool stopPinActive = false;   // debounced state of the stop input (HIGH = stop)
int stopPinCount = 0;
const int STOP_DEBOUNCE_SAMPLES = 3;  // ~150 ms at 20 Hz

const char* CSV_HEADER =
  "millis,pressure_hpa,altitude_m,temp_c,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
  "mx_uT,my_uT,mz_uT,tiltY_deg,tiltZ_deg,spin_deg,servo1_deg,servo2_deg,servo3_deg,servo4_deg,thrust_pct,mode";

// ---------- Latest readings (shared with web server) ----------
struct SensorSnapshot {
  unsigned long t = 0;
  float pressure_hpa = 0, altitude_m = 0, temp_c = 0;
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float mx = 0, my = 0, mz = 0;
  float tiltY = 0, tiltZ = 0, spin = 0;   // degrees (see updateOrientation)
  int servo_deg[4] = {90, 90, 90, 90};   // last commanded servo angles
} latest;

// ---------- Orientation for a VERTICALLY mounted board ----------
// tiltY : rotation about the sensor +Y axis (right-hand rule). Positive = the
//         top of the vehicle leans toward +Z.
// tiltZ : rotation about the sensor +Z axis. Positive = the top leans toward -Y.
// spin  : rotation about the vehicle's long axis (compass-style: clockwise seen
//         from above = positive). Gyro-integrated, gently pulled toward the
//         magnetometer heading so it doesn't drift away.
// Tilt is valid roughly +-90 deg from upright.
float filtTiltY = 0, filtTiltZ = 0, filtSpin = 0;
bool orientInit = false;
#define USE_MAG_SPIN_CORRECTION 1    // set to 0 to use the gyro only for spin
const float RAD2DEG = 57.29578;

// Smoothing (low-pass) on raw sensor values. Lower alpha = smoother but laggier.

// Gyro bias, measured once at boot while the board is held still.
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

float wrapAngle180(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz,
                        float mx, float my, float mz, float dt) {
  // Gravity reference. Upright, the accel reads (-1, 0, 0), so "up" = -X.
  float accTiltY = atan2(-az, -ax) * RAD2DEG;
  float accTiltZ = atan2( ay, -ax) * RAD2DEG;

  if (!orientInit) {            // start from the accel angles, not from zero
    filtTiltY = accTiltY;
    filtTiltZ = accTiltZ;
    orientInit = true;
  }

  // Gyro rate about Y moves tiltY, rate about Z moves tiltZ
  filtTiltY = COMP_ALPHA * (filtTiltY + gy * dt) + (1 - COMP_ALPHA) * accTiltY;
  filtTiltZ = COMP_ALPHA * (filtTiltZ + gz * dt) + (1 - COMP_ALPHA) * accTiltZ;

  // Spin about the long axis: rotation about +X looks clockwise from above
  filtSpin += gx * dt;

#if USE_MAG_SPIN_CORRECTION
  float un = sqrt(ax * ax + ay * ay + az * az);
  if (un > 0.05) {
    float ux = ax / un, uy = ay / un, uz = az / un;   // "up" in body axes
    // The AK8963 magnetometer axes are rotated relative to the accel/gyro axes
    float bx = my, by = mx, bz = -mz;
    // east = mag x up
    float ex = by * uz - bz * uy;
    float ey = bz * ux - bx * uz;
    float ez = bx * uy - by * ux;
    float en = sqrt(ex * ex + ey * ey + ez * ez);
    if (en > 2.0) {                                   // skip if there's no usable mag data
      ex /= en; ey /= en; ez /= en;
      float ny = uz * ex - ux * ez;                   // y component of north = up x east
      float magHeading = atan2(ey, ny) * RAD2DEG;     // heading of the body +Y axis
      float err = wrapAngle180(magHeading - wrapAngle180(filtSpin));
      filtSpin += (1 - COMP_ALPHA_SPIN) * err;        // gentle pull, no snapping
    }
  }
#endif

  latest.tiltY = filtTiltY;
  latest.tiltZ = filtTiltZ;
  latest.spin = wrapAngle180(filtSpin);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro -- keep the board completely still...");
  if (!imuOK) { Serial.println("Skipping gyro calibration: IMU not answering."); return; }
  setLedPattern(LED_GYRO_CAL);
  const int N = 400;                 // ~2 seconds
  float sumX = 0, sumY = 0, sumZ = 0;
  int good = 0;
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) { sumX += gx; sumY += gy; sumZ += gz; good++; }
    waitMs(5);
  }
  if (good < N / 2) { Serial.println("ERROR: IMU reads failed during calibration."); imuOK = false; return; }
  gyroBiasX = sumX / good;
  gyroBiasY = sumY / good;
  gyroBiasZ = sumZ / good;
  Serial.printf("Gyro bias measured: X=%.2f Y=%.2f Z=%.2f dps\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

// ---------- Low-level MPU9250 helpers ----------
void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

int mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;     // no ACK: the chip is not answering
  Wire.requestFrom((int)MPU_ADDR, (int)count);
  int n = 0;
  while (n < count && Wire.available()) dest[n++] = Wire.read();
  return n;
}

uint8_t akReadReg(uint8_t reg) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)AK8963_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

void akWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void akReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)AK8963_ADDR, (int)count);
  for (int i = 0; i < count && Wire.available(); i++) {
    dest[i] = Wire.read();
  }
}

void applyMpuConfig() {
  static const float aL[4] = {16384.0, 8192.0, 4096.0, 2048.0};   // LSB per g
  static const float gL[4] = {131.0, 65.5, 32.8, 16.4};           // LSB per deg/s
  int ar = constrain((int)cfg.accelRange, 0, 3), gr = constrain((int)cfg.gyroRange, 0, 3);
  mpuWriteReg(MPU_CONFIG, (uint8_t)constrain((int)cfg.dlpf, 0, 6));        // gyro low-pass
  mpuWriteReg(0x1D, (uint8_t)constrain((int)cfg.dlpf, 0, 6));              // accel low-pass
  mpuWriteReg(MPU_GYRO_CFG, (uint8_t)(gr << 3));
  mpuWriteReg(MPU_ACCEL_CFG, (uint8_t)(ar << 3));
  accelLsb = aL[ar];
  gyroLsb = gL[gr];
}

void applyBmp() {
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16,
                  (Adafruit_BMP280::sensor_filter)constrain((int)cfg.bmpFilter, 0, 4), Adafruit_BMP280::STANDBY_MS_63);
}

void loadCfg() {
  prefs.begin("tvc", true);
  if (prefs.getBytesLength("cfg") == sizeof(Cfg)) prefs.getBytes("cfg", &cfg, sizeof(Cfg));
  prefs.end();
}

void saveCfg() {
  prefs.begin("tvc", false);
  prefs.putBytes("cfg", &cfg, sizeof(Cfg));
  prefs.end();
}

bool initMPU9250() {
  uint8_t whoami = mpuReadReg(MPU_WHO_AM_I);
  Serial.print("MPU WHO_AM_I = 0x");
  Serial.println(whoami, HEX);
  // Genuine MPU9250 -> 0x71. Some clones/MPU6500-based boards report 0x70 or 0x73.
  imuOK = (whoami == 0x71 || whoami == 0x73 || whoami == 0x70 || whoami == 0x68);
  if (!imuOK) {
    Serial.println("ERROR: IMU not answering (expected WHO_AM_I 0x71). Check VCC/GND/SDA/SCL joints. Servos stay centred.");
    return false;
  }

  mpuWriteReg(MPU_PWR_MGMT_1, 0x00);   // wake up, clock = internal
  delay(100);
  applyMpuConfig();

  // Enable I2C bypass so AK8963 (magnetometer) is directly addressable
  mpuWriteReg(MPU_INT_PIN_CFG, 0x02);
  delay(10);

  uint8_t akWhoami = akReadReg(AK8963_WHO_AM_I);
  Serial.print("AK8963 WHO_AM_I = 0x");
  Serial.println(akWhoami, HEX); // should be 0x48

  akWriteReg(AK8963_CNTL1, 0x16); // continuous measurement mode 2, 16-bit output
  delay(10);

  return true;
}

bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t raw[14] = {0};
  if (mpuReadBytes(MPU_ACCEL_XOUT_H, 14, raw) != 14) return false;   // never use a half-filled buffer

  int16_t ax_raw = (raw[0] << 8) | raw[1];
  int16_t ay_raw = (raw[2] << 8) | raw[3];
  int16_t az_raw = (raw[4] << 8) | raw[5];
  // raw[6],raw[7] = temperature, skipped here
  int16_t gx_raw = (raw[8] << 8) | raw[9];
  int16_t gy_raw = (raw[10] << 8) | raw[11];
  int16_t gz_raw = (raw[12] << 8) | raw[13];

  // Scale factors for +-4g accel, +-500dps gyro (match config above)
  ax = ax_raw / accelLsb;   // g
  ay = ay_raw / accelLsb;
  az = az_raw / accelLsb;
  gx = gx_raw / gyroLsb;     // deg/s
  gy = gy_raw / gyroLsb;
  gz = gz_raw / gyroLsb;
  return true;
}

bool readMag(float &mx, float &my, float &mz) {
  uint8_t st1 = akReadReg(AK8963_ST1);
  if (!(st1 & 0x01)) return false; // data not ready

  uint8_t raw[7];
  akReadBytes(AK8963_XOUT_L, 7, raw); // last byte is ST2, must be read to latch data

  int16_t mx_raw = (raw[1] << 8) | raw[0];
  int16_t my_raw = (raw[3] << 8) | raw[2];
  int16_t mz_raw = (raw[5] << 8) | raw[4];

  // 16-bit mode sensitivity ~0.15 uT/LSB (nominal, uncalibrated)
  mx = mx_raw * 0.15;
  my = my_raw * 0.15;
  mz = mz_raw * 0.15;
  return true;
}

// ---------- Web control panel ----------
static const char PAGE[] = R"HTML(<!DOCTYPE html><html><head><title>Flight Panel</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body{font-family:monospace;background:#111;color:#ddd;margin:0;padding:12px;max-width:760px;margin:auto}
h2{color:#fff;margin:4px 0 10px} h3{margin:0 0 8px;color:#0f0;font-size:14px}
.card{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:10px;margin:10px 0}
.row{display:flex;flex-wrap:wrap;gap:6px;margin:6px 0}
button{background:#222;color:#0f0;border:1px solid #0a0;border-radius:6px;padding:10px 12px;font:inherit}
button:active{background:#063} button.on,button.sel{background:#063;color:#fff;border-color:#0f0}
.seg{display:flex;gap:6px;margin:6px 0}.seg button{flex:1}
.chip{display:inline-block;padding:4px 8px;border-radius:12px;margin:0 6px 6px 0;font-size:12px;background:#333}
.ok{background:#063;color:#0f0}.bad{background:#600;color:#fff}.warn{background:#640;color:#fc0}
.r{margin:8px 0} .r label{display:block;font-size:12px;color:#aaa} .r b{color:#0f0}
input[type=range]{width:100%}
canvas{width:100%;height:120px;background:#000;border-radius:4px}
.lg{font-size:11px;margin-top:4px}
.dis{opacity:.35;pointer-events:none}
.tag{font-size:11px;color:#fc0}
.jw{display:flex;gap:14px;align-items:flex-start;justify-content:space-around;flex-wrap:wrap}
.jl{font-size:11px;color:#aaa;text-align:center;margin-bottom:4px;max-width:170px}
.joy{position:relative;width:170px;height:170px;border-radius:50%;background:#000;border:2px solid #0a0;touch-action:none;opacity:.35}
.joy.act{opacity:1}
.knob{position:absolute;left:50%;top:50%;width:52px;height:52px;margin:-26px 0 0 -26px;border-radius:50%;background:#0f0;opacity:.8}
.scene{width:100px;height:220px;margin:40px auto;perspective:700px}
.cube{width:100px;height:220px;position:relative;transform-style:preserve-3d;transition:transform .1s linear}
.face{position:absolute;left:0;top:0;width:100px;height:220px;border:2px solid #0f0;background:rgba(0,255,0,.08);display:flex;align-items:center;justify-content:center;font-size:12px;color:#0f0;box-sizing:border-box}
.cap{height:100px;top:60px}.front{transform:translateZ(50px)}.back{transform:rotateY(180deg) translateZ(50px)}
.right{transform:rotateY(90deg) translateZ(50px)}.left{transform:rotateY(-90deg) translateZ(50px)}
.top{transform:rotateX(90deg) translateZ(110px);background:rgba(255,255,0,.15);border-color:#ff0;color:#ff0}
.bottom{transform:rotateX(-90deg) translateZ(110px)}
svg{width:100%;max-height:230px;background:#000;border-radius:6px} svg text{font-family:monospace}
.big{font-size:22px;color:#0f0}
</style></head><body>
<h2>Flight Control Panel</h2>
<div><span id=cLog class='chip bad'>LOG ?</span><span id=cMan class=chip>MANUAL OFF</span><span id=cArm class='chip ok'>SAFE</span><span id=cImu class='chip ok'>IMU ?</span><span id=cThr class=chip>THRUST OFF</span><span id=cMode class='chip warn'>MODE ?</span><span id=cNet class='chip warn'>connecting</span></div>
<div class=seg><button id=bM0>MODE 1: TVC + AUX FINS</button><button id=bM1>MODE 2: 4 FINS</button></div>
<div class=row><button id=bCal>Calibrate gyro</button><button id=bMan>Manual servos: OFF</button><button id=bLog>Stop logging</button><button id=bSave>Save settings</button><button id=bDef>Defaults</button></div>
<div class=card><h3>Manual servo control (enable manual first)</h3>
<div class=jw><div><div class=jl id=jl1></div><div class=joy id=joy1><div class=knob id=knob1></div></div></div>
<div><div class=jl id=jl2></div><div class=joy id=joy2><div class=knob id=knob2></div></div></div></div>
<div id=jv style='font-size:12px;color:#888;margin-top:6px'>J1 0,0   J2 0,0</div></div>
<div class=card><h3>Orientation - avionics unit</h3>
<div class=scene><div class=cube id=cube><div class='face front'>+Y</div><div class='face back'>-Y</div><div class='face right'>+Z</div><div class='face left'>-Z</div><div class='face cap top'>TOP -X</div><div class='face cap bottom'>+X</div></div></div></div>
<div class=card id=vTVC><h3>Mode 1 - TVC mount position (servo deflection)</h3><svg id=svgTVC viewBox="0 0 240 150"></svg></div>
<div class=card id=vAux><h3>Mode 1 - aux fins / canards position</h3><svg id=svgAux viewBox="0 0 240 100"></svg></div>
<div class=card id=vFins><h3>Mode 2 - fin positions (bottom view, plus layout)</h3><svg id=svgFins viewBox="0 0 240 215"></svg></div>
<div class=card id=cardThr><h3>Thrust (BLDC / ESC)</h3>
<div class=row><button id=bTE>Thrust enable: OFF</button><button id=bTA>Source: MANUAL</button><button id=bCut style='color:#f66;border-color:#a00'>CUT THRUST</button></div>
<div class=r><label>Manual thrust: <b id=thrv>0</b>%   |   actual output: <b class=big id=thra>0</b>%</label><input type=range id=thrSl min=0 max=100 step=1 value=0></div>
<div class=row><button id=bAlt>Hold current altitude</button></div>
<p class=lg style='color:#888'>Output stays at zero unless the hardware ARM switch is ON and Thrust enable is ON. Manual thrust cuts to zero if this page goes quiet for 2 s. AUTO settings are in the list below.</p></div>
<div id=charts></div>
<div id=settings></div>
<p style='color:#666;font-size:12px'>Changes apply instantly. "Save settings" keeps them across power cycles. Keep the vehicle still while calibrating.</p>
<script>
const $=id=>document.getElementById(id);
const get=async u=>{try{const r=await fetch(u);return await r.json()}catch(e){return null}};
const cmd=(c,v)=>get('/cmd?c='+c+(v===undefined?'':'&v='+v));
const SGN={tiltY:1,tiltZ:1,spin:1};
const NM=[['TVC pitch (S1)','TVC yaw (S2)','Aux 1 (S3)','Aux 2 (S4)'],['Fin A (S1)','Fin B (S2)','Fin C (S3)','Fin D (S4)']];
//P-BEGIN
const P=[];const add=(k,l,mn,mx,st,g,t)=>P.push([k,l,mn,mx,st,g,t]);
const sp=(m,g)=>{for(let i=0;i<4;i++){const n=NM[m][i];add('m'+m+'t'+i,n+' trim (deg)',-45,45,.5,g,m);add('m'+m+'n'+i,n+' min angle',0,90,1,g,m);add('m'+m+'x'+i,n+' max angle',90,180,1,g,m)}};
const G1='MODE 1 - TVC mount + aux fins gains',G1s='MODE 1 - servo trim and travel',G2='MODE 2 - four fin gains',G2s='MODE 2 - fin trim and travel';
add('kp','TVC Kp (servo deg per deg of tilt)',0,10,.05,G1,0);add('kd','TVC Kd (damping)',0,2,.01,G1,0);add('maxDef','TVC max deflection (deg)',0,90,1,G1,0);
add('auxFunc','Aux fins function',0,1,1,G1,0);
add('auxKp','Aux pitch/yaw Kp (0 = parked)',0,10,.05,G1,0);add('auxKd','Aux pitch/yaw Kd',0,2,.01,G1,0);
add('auxRollKp','Aux roll Kp (angle)',0,10,.05,G1,0);add('auxRollKd','Aux roll Kd (rate)',0,2,.01,G1,0);add('auxMax','Aux max deflection (deg)',0,90,1,G1,0);
sp(0,G1s);
add('finKp','Fin Kp (pitch/yaw)',0,10,.05,G2,1);add('finKd','Fin Kd (pitch/yaw)',0,2,.01,G2,1);
add('finRollKp','Fin roll Kp (angle)',0,10,.05,G2,1);add('finRollKd','Fin roll Kd (rate)',0,2,.01,G2,1);add('finMax','Fin max deflection (deg)',0,90,1,G2,1);
sp(1,G2s);
add('tiltTrimY','Tilt trim Y (deg)',-30,30,.1,'Mounting offset',-1);add('tiltTrimZ','Tilt trim Z (deg)',-30,30,.1,'Mounting offset',-1);
add('accelAlpha','Accel smoothing alpha (1 = none)',.02,1,.01,'Filtering',-1);add('gyroAlpha','Gyro smoothing alpha (1 = none)',.02,1,.01,'Filtering',-1);
add('compAlpha','Tilt: gyro trust',.8,.999,.001,'Filtering',-1);add('compSpin','Spin: gyro trust',.8,.999,.001,'Filtering',-1);
add('dlpf','IMU low-pass filter',0,6,1,'Sensors',-1);add('accelRange','Accel range',0,3,1,'Sensors',-1);add('gyroRange','Gyro range',0,3,1,'Sensors',-1);add('bmpFilter','Barometer filter',0,4,1,'Sensors',-1);
add('sampleMs','Sample / control interval (ms)',10,200,1,'Sampling',-1);
add('thrMethod','AUTO thrust method',0,1,1,'AUTO thrust',-1);add('hoverThr','Hover thrust (%)',0,100,1,'AUTO thrust',-1);add('tiltComp','Tilt compensation',0,1,1,'AUTO thrust',-1);
add('altKp','Altitude hold Kp (% per m)',0,20,.1,'AUTO thrust',-1);add('altKd','Altitude hold Kd (% per m/s)',0,30,.1,'AUTO thrust',-1);add('altTarget','Target altitude (m)',-500,10000,1,'AUTO thrust',-1);
add('thrMin','AUTO min thrust (%)',0,100,1,'AUTO thrust',-1);add('thrMax','Max thrust, all modes (%)',0,100,1,'AUTO thrust',-1);add('thrRamp','Ramp-up rate (% per s)',5,200,1,'AUTO thrust',-1);
//P-END
const EN={dlpf:['250Hz','184Hz','92Hz','41Hz','20Hz','10Hz','5Hz'],accelRange:['+-2g','+-4g','+-8g','+-16g'],gyroRange:['+-250','+-500','+-1000','+-2000 dps'],bmpFilter:['off','x2','x4','x8','x16'],auxFunc:['pitch/yaw assist','roll control'],thrMethod:['fixed hover %','altitude hold'],tiltComp:['off','on']};
let C={},tm={},last={},mode=0,modeLock=0,man=false,busy=false;
const show=(k,v)=>{const e=$('v_'+k);if(e)e.textContent=EN[k]?EN[k][Math.round(v)]:(+v).toFixed(3).replace(/\.?0+$/,'')};
function dirBtns(){for(let m=0;m<2;m++)for(let i=0;i<4;i++){const b=$('bd'+m+i);if(b)b.textContent=NM[m][i]+' dir: '+(C['m'+m+'d'+i]<0?'REVERSED':'normal')}}
function applyMode(m){mode=m;$('bM0').className=m==0?'sel':'';$('bM1').className=m==1?'sel':'';
 document.querySelectorAll('.g0').forEach(e=>e.classList.toggle('dis',m!=0));document.querySelectorAll('.g1').forEach(e=>e.classList.toggle('dis',m!=1));
 document.querySelectorAll('.tag0').forEach(e=>e.textContent=m==0?'(ACTIVE)':'(inactive)');document.querySelectorAll('.tag1').forEach(e=>e.textContent=m==1?'(ACTIVE)':'(inactive)');
 $('vTVC').style.display=m==0?'block':'none';$('vAux').style.display=m==0?'block':'none';$('vFins').style.display=m==1?'block':'none';
 $('jl1').textContent=m==0?'TVC mount: X = pitch (S1), Y = yaw (S2)':'Fins: X = Fin A (S1), Y = Fin B (S2)';
 $('jl2').textContent=m==0?'Aux fins: X = Aux 1 (S3), Y = Aux 2 (S4)':'Fins: X = Fin C (S3), Y = Fin D (S4)';
 $('cMode').textContent=m==0?'MODE 1: TVC + AUX':'MODE 2: 4 FINS'}
function build(){const G={};for(const p of P)(G[p[5]]=G[p[5]]||[]).push(p);let h='';
 for(const g in G){const t=G[g][0][6];h+='<div class="card g'+t+'"><h3>'+g+(t>=0?' <span class="tag tag'+t+'"></span>':'')+'</h3>';
  if(/trim and travel/.test(g)){h+='<div class=row>';for(let i=0;i<4;i++)h+='<button id=bd'+t+i+'></button>';h+='</div>'}
  for(const p of G[g])h+='<div class=r><label>'+p[1]+': <b id=v_'+p[0]+'></b></label><input type=range id=i_'+p[0]+' min='+p[2]+' max='+p[3]+' step='+p[4]+'></div>';h+='</div>'}
 $('settings').innerHTML=h;
 for(const p of P){const k=p[0],i=$('i_'+k);i.value=C[k];show(k,C[k]);i.oninput=()=>{show(k,i.value);clearTimeout(tm[k]);tm[k]=setTimeout(()=>get('/set?'+k+'='+i.value),150)}}
 for(let m=0;m<2;m++)for(let i=0;i<4;i++){const b=$('bd'+m+i);if(b)b.onclick=()=>{const k='m'+m+'d'+i;C[k]=-C[k];get('/set?'+k+'='+C[k]);dirBtns()}}
 dirBtns();applyMode(mode)}
async function load(){const c=await get('/cfg');if(c){C=c;mode=Math.round(C.mode);build()}}
// ---- position views
const PV={};
const fin=(id,cx,cy,lab)=>{PV[id]=[cx,cy];return '<circle cx='+cx+' cy='+cy+' r=27 fill=none stroke=#333 /><line x1='+cx+' y1='+(cy-26)+' x2='+cx+' y2='+(cy+26)+' stroke=#264 stroke-dasharray=2,3 /><g id='+id+'><ellipse cx='+cx+' cy='+cy+' rx=3.5 ry=22 fill=#0f0 /></g><text x='+cx+' y='+(cy+42)+' text-anchor=middle fill=#aaa font-size=10 id=t_'+id+'>'+lab+'</text>'};
const mount=(id,cx,lab)=>{PV[id]=[cx,34];return '<rect x='+(cx-45)+' y=8 width=90 height=6 fill=#555 /><g id='+id+'><rect x='+(cx-14)+' y=34 width=28 height=36 fill=#345 stroke=#0f0 /><polygon points="'+(cx-9)+',70 '+(cx+9)+',70 '+(cx+17)+',100 '+(cx-17)+',100" fill=#0a6 /><line x1='+cx+' y1=100 x2='+cx+' y2=128 stroke=#fa0 stroke-width=3 /></g><circle cx='+cx+' cy=34 r=4 fill=#fff /><text x='+cx+' y=146 text-anchor=middle fill=#aaa font-size=10 id=t_'+id+'>'+lab+'</text>'};
$('svgTVC').innerHTML=mount('mP',60,'Pitch S1')+mount('mY',180,'Yaw S2');
$('svgAux').innerHTML=fin('aA',60,42,'Aux 1 S3')+fin('aB',180,42,'Aux 2 S4');
$('svgFins').innerHTML='<circle cx=120 cy=108 r=22 fill=none stroke=#555 /><text x=120 y=112 text-anchor=middle fill=#666 font-size=9>EXHAUST</text>'+fin('fA',120,30,'Fin A')+fin('fB',200,108,'Fin B')+fin('fC',120,186,'Fin C')+fin('fD',40,108,'Fin D');
function views(d){const m=mode,dev=i=>d['s'+(i+1)]-90-(C['m'+m+'t'+i]||0);
 const set=(id,v,l)=>{const a=PV[id];$(id).setAttribute('transform','rotate('+v+' '+a[0]+' '+a[1]+')');$('t_'+id).textContent=l+' '+(v>=0?'+':'')+v.toFixed(0)+' deg'};
 if(m==0){set('mP',dev(0),'Pitch S1');set('mY',dev(1),'Yaw S2');set('aA',dev(2),'Aux 1 S3');set('aB',dev(3),'Aux 2 S4')}
 else{set('fA',dev(0),'Fin A');set('fB',dev(1),'Fin B');set('fC',dev(2),'Fin C');set('fD',dev(3),'Fin D')}}
// ---- charts (unchanged, plus servos 3/4 and thrust)
const CH=[['Tilt / spin (deg)',['tiltY','tiltZ','spin']],['Gyro (deg/s)',['gx','gy','gz']],['Accel (g)',['ax','ay','az']],['Altitude (m)',['altitude_m']],['Servo outputs 1 / 2 (deg)',['s1','s2']],['Servo outputs 3 / 4 (deg)',['s3','s4']],['Thrust output (%)',['thr']]];
const COL=['#0f0','#0af','#fa0'],N=160,buf={};
{let h='';CH.forEach((c,i)=>h+='<div class=card><h3>'+c[0]+'</h3><canvas id=ch'+i+' width=600 height=120></canvas><div class=lg id=lg'+i+'></div></div>');$('charts').innerHTML=h}
function draw(i){const ks=CH[i][1],cv=$('ch'+i),g=cv.getContext('2d'),W=cv.width,H=cv.height;g.clearRect(0,0,W,H);
 let lo=1e9,hi=-1e9;ks.forEach(k=>(buf[k]||[]).forEach(v=>{if(v<lo)lo=v;if(v>hi)hi=v}));
 if(hi-lo<1e-6)hi=lo+1;const pd=(hi-lo)*.1;lo-=pd;hi+=pd;
 g.strokeStyle='#333';g.beginPath();g.moveTo(0,H/2);g.lineTo(W,H/2);g.stroke();
 ks.forEach((k,j)=>{g.strokeStyle=COL[j];g.beginPath();(buf[k]||[]).forEach((v,x)=>{const px=x*W/(N-1),py=H-(v-lo)/(hi-lo)*H;x?g.lineTo(px,py):g.moveTo(px,py)});g.stroke()});
 g.fillStyle='#888';g.font='11px monospace';g.fillText(hi.toFixed(1),2,10);g.fillText(lo.toFixed(1),2,H-2);
 $('lg'+i).innerHTML=ks.map((k,j)=>{const a=buf[k]||[];return '<span style="color:'+COL[j]+'">'+k+' '+(a.length?a[a.length-1].toFixed(2):'-')+'</span>'}).join(' &nbsp; ')}
// ---- polling
async function poll(){if(busy)return;busy=true;const d=await get('/data');busy=false;
 if(!d){$('cNet').textContent='offline';$('cNet').className='chip bad';return}
 $('cNet').textContent='connected';$('cNet').className='chip ok';last=d;man=!!d.man;
 if(Date.now()>modeLock&&d.mode!==mode)applyMode(d.mode);
 for(const k in d){(buf[k]=buf[k]||[]).push(d[k]);if(buf[k].length>N)buf[k].shift()}
 CH.forEach((c,i)=>draw(i));views(d);
 $('cube').style.transform='rotateY('+(-SGN.spin*d.spin)+'deg) rotateZ('+(SGN.tiltY*d.tiltY)+'deg) rotateX('+(SGN.tiltZ*d.tiltZ)+'deg)';
 const l=$('cLog');l.textContent=d.logging?'SD LOGGING ON':(d.pin?'LOGGING OFF (switch)':d.ws?'LOGGING OFF (web)':'LOGGING OFF (no card / error)');l.className='chip '+(d.logging?'ok':'bad');
 $('cMan').textContent=man?'MANUAL ON':'MANUAL OFF';$('cMan').className='chip '+(man?'warn':'');
 $('cImu').textContent=d.imu?'IMU OK':'IMU FAULT';$('cImu').className='chip '+(d.imu?'ok':'bad');
 $('cArm').textContent=d.arm?'ARMED':'SAFE';$('cArm').className='chip '+(d.arm?'bad':'ok');
 const t=$('cThr');if(!d.thrEn){t.textContent='THRUST OFF';t.className='chip'}else if(!d.arm){t.textContent='THRUST ON - ARM SWITCH OFF';t.className='chip warn'}else{t.textContent='THRUST LIVE '+d.thr.toFixed(0)+'%';t.className='chip bad'}
 $('bTE').textContent='Thrust enable: '+(d.thrEn?'ON':'OFF');$('bTE').className=d.thrEn?'sel':'';
 $('bTA').textContent='Source: '+(d.thrAuto?'AUTO':'MANUAL');$('bTA').className=d.thrAuto?'sel':'';
 $('thra').textContent=d.thr.toFixed(0);if(!d.thrEn){$('thrSl').value=0;$('thrv').textContent=0}
 $('bMan').textContent='Manual servos: '+(man?'ON':'OFF');$('bMan').className=man?'on':'';$('joy1').className='joy '+(man?'act':'');$('joy2').className='joy '+(man?'act':'');
 $('bLog').textContent=d.ws?'Resume logging':'Stop logging'}
setInterval(poll,120);
// ---- buttons
$('bM0').onclick=()=>{modeLock=Date.now()+1500;C.mode=0;get('/set?mode=0');applyMode(0)};
$('bM1').onclick=()=>{modeLock=Date.now()+1500;C.mode=1;get('/set?mode=1');applyMode(1)};
$('bCal').onclick=async()=>{if(!confirm('Keep the vehicle completely still, then OK.'))return;$('bCal').textContent='Calibrating...';await cmd('calgyro');$('bCal').textContent='Calibrate gyro'};
$('bMan').onclick=()=>cmd('manual',man?0:1);
$('bLog').onclick=()=>cmd('logstop',last.ws?0:1);
$('bSave').onclick=async()=>{await cmd('save');$('bSave').textContent='Saved!';setTimeout(()=>$('bSave').textContent='Save settings',1200)};
$('bDef').onclick=async()=>{if(!confirm('Reset all settings to defaults?'))return;await cmd('defaults');load()};
$('bTE').onclick=()=>{if(last.thrEn){cmd('thren',0);return}if(!confirm('Enable thrust output? Propeller / fan clear of people?'))return;$('thrSl').value=0;$('thrv').textContent=0;cmd('thren',1)};
$('bTA').onclick=()=>{if(!last.thrAuto&&!confirm('Switch to AUTO thrust?'))return;cmd('thrauto',last.thrAuto?0:1)};
$('bCut').onclick=()=>{$('thrSl').value=0;$('thrv').textContent=0;cmd('cut')};
$('bAlt').onclick=()=>cmd('altnow').then(load);
$('thrSl').oninput=()=>{$('thrv').textContent=$('thrSl').value;get('/cmd?c=thr&v='+$('thrSl').value)};
setInterval(()=>{if(last.thrEn&&!last.thrAuto)get('/cmd?c=thr&v='+$('thrSl').value)},150);   // heartbeat for the 2 s dead-man timer
// ---- joysticks
function mkJoy(el,kn){const o={x:0,y:0,drag:false};
 const up=e=>{const r=el.getBoundingClientRect(),R=r.width/2;let x=(e.clientX-r.left-R)/R,y=-(e.clientY-r.top-R)/R;const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}o.x=x;o.y=y;kn.style.left=(50+x*40)+'%';kn.style.top=(50-y*40)+'%'};
 el.onpointerdown=e=>{if(!man)return;o.drag=true;el.setPointerCapture(e.pointerId);up(e)};
 el.onpointermove=e=>{if(o.drag)up(e)};
 el.onpointerup=el.onpointercancel=()=>{o.drag=false;o.x=o.y=0;kn.style.left=kn.style.top='50%'};return o}
const J1=mkJoy($('joy1'),$('knob1')),J2=mkJoy($('joy2'),$('knob2'));
setInterval(()=>{if(man){get('/joy?x='+J1.x.toFixed(3)+'&y='+J1.y.toFixed(3)+'&u='+J2.x.toFixed(3)+'&v='+J2.y.toFixed(3));$('jv').textContent='J1 '+J1.x.toFixed(2)+','+J1.y.toFixed(2)+'   J2 '+J2.x.toFixed(2)+','+J2.y.toFixed(2)}},80);
load();
</script></body></html>)HTML";

// Settings table: key, variable, min, max, flags (1 = integer, 2 = hardware setting that must be re-applied)
struct Param { const char* key; float* val; float lo, hi; uint8_t flags; };
Param params[100];
char keyBuf[100][12];
int NPARAMS = 0;

void addParam(const char* key, float* v, float lo, float hi, uint8_t flags = 0) {
  if (NPARAMS >= 100) return;
  int n = NPARAMS++;
  strncpy(keyBuf[n], key, 11); keyBuf[n][11] = 0;
  params[n] = Param{keyBuf[n], v, lo, hi, flags};
}

void buildParams() {
  addParam("mode", &cfg.mode, 0, 1, 1);
  for (int m = 0; m < 2; m++) for (int i = 0; i < 4; i++) {          // per mode, per servo
    char k[12];
    snprintf(k, sizeof k, "m%dt%d", m, i); addParam(k, &cfg.sv[m][i].trim, -45, 45);
    snprintf(k, sizeof k, "m%dd%d", m, i); addParam(k, &cfg.sv[m][i].dir, -1, 1, 1);
    snprintf(k, sizeof k, "m%dn%d", m, i); addParam(k, &cfg.sv[m][i].mn, 0, 90);
    snprintf(k, sizeof k, "m%dx%d", m, i); addParam(k, &cfg.sv[m][i].mx, 90, 180);
  }
  addParam("kp", &cfg.kp, 0, 10);           addParam("kd", &cfg.kd, 0, 2);          addParam("maxDef", &cfg.maxDef, 0, 90);
  addParam("auxFunc", &cfg.auxFunc, 0, 1, 1);
  addParam("auxKp", &cfg.auxKp, 0, 10);     addParam("auxKd", &cfg.auxKd, 0, 2);    addParam("auxMax", &cfg.auxMax, 0, 90);
  addParam("auxRollKp", &cfg.auxRollKp, 0, 10); addParam("auxRollKd", &cfg.auxRollKd, 0, 2);
  addParam("finKp", &cfg.finKp, 0, 10);     addParam("finKd", &cfg.finKd, 0, 2);    addParam("finMax", &cfg.finMax, 0, 90);
  addParam("finRollKp", &cfg.finRollKp, 0, 10); addParam("finRollKd", &cfg.finRollKd, 0, 2);
  addParam("tiltTrimY", &cfg.tiltTrimY, -30, 30); addParam("tiltTrimZ", &cfg.tiltTrimZ, -30, 30);
  addParam("accelAlpha", &cfg.accelAlpha, 0.02, 1); addParam("gyroAlpha", &cfg.gyroAlpha, 0.02, 1);
  addParam("compAlpha", &cfg.compAlpha, 0.8, 0.999); addParam("compSpin", &cfg.compSpin, 0.8, 0.999);
  addParam("sampleMs", &cfg.sampleMs, 10, 200, 1);
  addParam("dlpf", &cfg.dlpf, 0, 6, 3);     addParam("accelRange", &cfg.accelRange, 0, 3, 3);
  addParam("gyroRange", &cfg.gyroRange, 0, 3, 3); addParam("bmpFilter", &cfg.bmpFilter, 0, 4, 3);
  addParam("thrMethod", &cfg.thrMethod, 0, 1, 1); addParam("hoverThr", &cfg.hoverThr, 0, 100); addParam("tiltComp", &cfg.tiltComp, 0, 1, 1);
  addParam("altKp", &cfg.altKp, 0, 20);     addParam("altKd", &cfg.altKd, 0, 30);   addParam("altTarget", &cfg.altTarget, -500, 10000);
  addParam("thrMin", &cfg.thrMin, 0, 100);  addParam("thrMax", &cfg.thrMax, 0, 100); addParam("thrRamp", &cfg.thrRamp, 5, 200);
}

void handleRoot() { server.send(200, "text/html", PAGE); }

void handleCfg() {
  String j = "{";
  for (int i = 0; i < NPARAMS; i++) {
    if (i) j += ",";
    j += "\"" + String(params[i].key) + "\":" + String(*params[i].val, 3);
  }
  server.send(200, "application/json", j + "}");
}

void handleSet() {
  bool hw = false;
  for (int i = 0; i < NPARAMS; i++) {
    if (!server.hasArg(params[i].key)) continue;
    float v = constrain(server.arg(params[i].key).toFloat(), params[i].lo, params[i].hi);
    if (params[i].flags & 1) v = roundf(v);
    *params[i].val = v;
    if (params[i].flags & 2) hw = true;
  }
  if (hw) { applyMpuConfig(); applyBmp(); }
  server.send(200, "application/json", "{\"ok\":1}");
}

void handleCmd() {
  String c = server.arg("c");
  int v = server.arg("v").toInt();
  if (c == "calgyro") calibrateGyro();                 // blocks ~2 s; vehicle must be still
  else if (c == "manual") { manualMode = (v != 0); joyX = joyY = 0; manualLastMs = millis(); }
  else if (c == "logstop") webStop = (v != 0);
  else if (c == "save") saveCfg();
  else if (c == "defaults") { cfg = Cfg(); applyMpuConfig(); applyBmp(); }
  else if (c == "thren") { thrEnable = (v != 0); thrManual = 0; thrBeatMs = millis(); }
  else if (c == "thrauto") { thrAuto = (v != 0); thrManual = 0; thrBeatMs = millis(); }
  else if (c == "thr") { thrManual = constrain(v, 0, 100); thrBeatMs = millis(); }
  else if (c == "altnow") { cfg.altTarget = latest.altitude_m; }
  else if (c == "cut") { thrEnable = false; thrManual = 0; }
  server.send(200, "application/json", "{\"ok\":1}");
}

void handleJoy() {
  joyX = constrain(server.arg("x").toFloat(), -1.0, 1.0);
  joyY = constrain(server.arg("y").toFloat(), -1.0, 1.0);
  joy2X = constrain(server.arg("u").toFloat(), -1.0, 1.0);
  joy2Y = constrain(server.arg("v").toFloat(), -1.0, 1.0);
  manualLastMs = millis();
  server.send(200, "application/json", "{\"ok\":1}");
}

void handleData() {
  String json = "{";
  json += "\"t\":" + String(latest.t) + ",";
  json += "\"pressure_hpa\":" + String(latest.pressure_hpa, 2) + ",";
  json += "\"altitude_m\":" + String(latest.altitude_m, 2) + ",";
  json += "\"temp_c\":" + String(latest.temp_c, 2) + ",";
  json += "\"ax\":" + String(latest.ax, 3) + ",";
  json += "\"ay\":" + String(latest.ay, 3) + ",";
  json += "\"az\":" + String(latest.az, 3) + ",";
  json += "\"gx\":" + String(latest.gx, 2) + ",";
  json += "\"gy\":" + String(latest.gy, 2) + ",";
  json += "\"gz\":" + String(latest.gz, 2) + ",";
  json += "\"mx\":" + String(latest.mx, 1) + ",";
  json += "\"my\":" + String(latest.my, 1) + ",";
  json += "\"mz\":" + String(latest.mz, 1) + ",";
  json += "\"tiltY\":" + String(latest.tiltY, 1) + ",";
  json += "\"tiltZ\":" + String(latest.tiltZ, 1) + ",";
  json += "\"spin\":" + String(latest.spin, 1) + ",";
  json += "\"s1\":" + String(latest.servo_deg[0]) + ",";
  json += "\"s2\":" + String(latest.servo_deg[1]) + ",";
  json += "\"s3\":" + String(latest.servo_deg[2]) + ",";
  json += "\"s4\":" + String(latest.servo_deg[3]) + ",";
  json += "\"thr\":" + String(thrActual, 1) + ",";
  json += "\"thrEn\":" + String(thrEnable ? 1 : 0) + ",";
  json += "\"thrAuto\":" + String(thrAuto ? 1 : 0) + ",";
  json += "\"mode\":" + String(modeIdx()) + ",";
  json += "\"logging\":" + String(logActive ? 1 : 0) + ",";
  json += "\"pin\":" + String(stopPinActive ? 1 : 0) + ",";
  json += "\"ws\":" + String(webStop ? 1 : 0) + ",";
  json += "\"man\":" + String(manualMode ? 1 : 0) + ",";
  json += "\"arm\":" + String(payloadArmed ? 1 : 0) + ",";
  json += "\"imu\":" + String(imuOK ? 1 : 0);
  json += "}";
  server.send(200, "application/json", json);
}

void setupWiFiServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. Connect phone to WiFi '");
  Serial.print(AP_SSID);
  Serial.println("', then browse to:");
  Serial.println(WiFi.softAPIP()); // normally 192.168.4.1

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cfg", handleCfg);
  server.on("/set", handleSet);
  server.on("/cmd", handleCmd);
  server.on("/joy", handleJoy);
  server.begin();
}

// ---------- SD card logger ----------
bool mountSD() {
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdMounted = SD.begin(SD_CS_PIN, sdSPI);
  if (!sdMounted) {
    Serial.println("ERROR: SD card init failed. Check wiring/CS pin/card format (must be FAT32).");
  }
  return sdMounted;
}

bool startLogging() {
  if (!sdMounted) return false;

  if (!logFileCreated) {
    // New numbered file each boot so earlier flights are never overwritten
    for (int i = 1; i < 1000; i++) {
      snprintf(logFilename, sizeof(logFilename), "/flight_%03d.csv", i);
      if (!SD.exists(logFilename)) break;
    }
  }
  bool needHeader = !logFileCreated || !SD.exists(logFilename);

  logFile = SD.open(logFilename, needHeader ? FILE_WRITE : FILE_APPEND);
  if (!logFile) {
    Serial.println("ERROR: could not open log file.");
    return false;
  }
  if (needHeader) {
    logFile.println(CSV_HEADER);
    logFile.flush();
  }
  logFileCreated = true;
  logActive = true;
  samplesSinceFlush = 0;
  Serial.printf("Logging ON -> %s\n", logFilename);
  return true;
}

void stopLogging(const char* reason) {
  if (!logActive) return;
  logFile.flush();
  logFile.close();
  logActive = false;
  Serial.printf("Logging OFF (%s). File closed - safe to remove the card.\n", reason);
}

// One attempt to (re)start logging; remounts the card first if the simple
// reopen fails, in case the card was removed and reinserted while stopped.
void resumeLogging() {
  if (startLogging()) return;
  SD.end();
  if (mountSD()) startLogging();
}

void setupSD() {
  if (!mountSD()) {
    Serial.println("Continuing WITHOUT logging. Toggle the stop input to retry.");
    return;
  }
  if (stopPinActive) {
    Serial.println("Stop input is HIGH at boot - logging held OFF.");
    return;
  }
  startLogging();
}

// Debounced read of the stop input (HIGH = stop logging)
void updateStopPin() {
  bool raw = (digitalRead(LOG_STOP_PIN) == HIGH);
  if (raw != stopPinActive) {
    if (++stopPinCount >= STOP_DEBOUNCE_SAMPLES) {
      stopPinActive = raw;
      stopPinCount = 0;
    }
  } else {
    stopPinCount = 0;
  }
}

// ARM switch (HIGH = armed), debounced ~50 ms. If it is already ON at power-up it
// is ignored (fast-blinking ARMED LED) until it has been switched OFF once.
void updateArmed() {
  static bool lastRaw = false, sw = false;
  static unsigned long changedAt = 0;
  bool raw = (digitalRead(ARM_IN_PIN) == HIGH);
  if (raw != lastRaw) { lastRaw = raw; changedAt = millis(); }
  if (millis() - changedAt > 50) sw = raw;

  if (!sw) armLockout = false;
  payloadArmed = sw && !armLockout;

  if (sw && armLockout) {                       // warning: cycle the switch
    digitalWrite(ARMED_LED_PIN, (millis() % 200) < 100 ? HIGH : LOW);
  } else {
    digitalWrite(ARMED_LED_PIN, payloadArmed ? HIGH : LOW);
  }
}

// Green LED solid when ready; SD LED blinks fast while logging, slow when not;
// ARMED LED follows the ARM switch.
void updateIndicators() {
  if (imuOK) setLedPattern(LED_READY_SOLID);
  else       setLedPattern(LED_IMU_FAULT);
  updateLed();
  if (logActive) setSdLedPattern(SDLED_LOGGING);
  else           setSdLedPattern(SDLED_NOT_LOGGING);
  updateSdLed();
  updateArmed();
  digitalWrite(SPARE_LED_PIN, ((millis() / 500) % 2) ? HIGH : LOW);   // spare red: blink every half second
}

// ---------- Servo output ----------
int modeIdx() { return cfg.mode < 0.5 ? 0 : 1; }

// off = degrees from the (trimmed) centre. Direction is NOT applied here.
void setServoRaw(int i, float off) {
  const ServoCfg &c = cfg.sv[modeIdx()][i];
  int a = constrain(90 + (int)lroundf(c.trim) + (int)lroundf(off), (int)c.mn, (int)c.mx);
  SV[i]->write(a);
  latest.servo_deg[i] = a;
}
void setServo(int i, float f) { setServoRaw(i, (cfg.sv[modeIdx()][i].dir < 0 ? -1 : 1) * f); }
void holdServosCentre() { for (int i = 0; i < 4; i++) setServoRaw(i, 0); }
float clampf(float v, float lim) { return constrain(v, -lim, lim); }

// ---------- One-time servo startup tests ----------
// Each servo in turn between its own min and max travel limits: lo -> hi -> lo -> hi.
void sweepServoTwice(int i) {
  const ServoCfg &c = cfg.sv[modeIdx()][i];
  Serial.printf("Testing servo %d on GPIO %d...\n", i + 1, SERVO_PINS[i]);
  SV[i]->write((int)c.mn); waitMs(700);   // first move may start from an unknown position
  SV[i]->write((int)c.mx); waitMs(500);
  SV[i]->write((int)c.mn); waitMs(500);
  SV[i]->write((int)c.mx); waitMs(500);
}

void runServoStartupSequence() {      // servos 1 and 2 (TVC mount, or fins A and B)
  Serial.println("Running servo test 1/2...");
  sweepServoTwice(0);
  sweepServoTwice(1);
}

void runCanardTest() {                // servos 3 and 4 (aux fins, or fins C and D)
  Serial.println("Running servo test 3/4...");
  sweepServoTwice(2);
  sweepServoTwice(3);
  holdServosCentre();
  Serial.println("Servo tests complete.");
}

// ---------- Automatic control ----------
// Every servo pushes AGAINST the error it is responsible for (proportional + rate damping).
//  MODE 1: servo 1 <- tilt Y, servo 2 <- tilt Z (TVC mount); servos 3/4 = aux fins, either
//          pitch/yaw assist or roll control (two fins deflect in opposite directions).
//  MODE 2: plus layout. A+C pitch pair (tilt Y), B+D yaw pair (tilt Z), all four twist for roll.
void updateServosAuto() {
  float eY = latest.tiltY - cfg.tiltTrimY, eZ = latest.tiltZ - cfg.tiltTrimZ;
  float spinErr = wrapAngle180(latest.spin - spinRef);
  if (cfg.mode < 0.5) {
    setServo(0, clampf(-(cfg.kp * eY + cfg.kd * latest.gy), cfg.maxDef));
    setServo(1, clampf(-(cfg.kp * eZ + cfg.kd * latest.gz), cfg.maxDef));
    if (cfg.auxFunc < 0.5) {                              // aux: pitch/yaw assist
      setServo(2, clampf(-(cfg.auxKp * eY + cfg.auxKd * latest.gy), cfg.auxMax));
      setServo(3, clampf(-(cfg.auxKp * eZ + cfg.auxKd * latest.gz), cfg.auxMax));
    } else {                                              // aux: roll control, differential
      float r = clampf(-(cfg.auxRollKp * spinErr + cfg.auxRollKd * latest.gx), cfg.auxMax);
      setServo(2, r);
      setServo(3, -r);
    }
  } else {
    float cy = -(cfg.finKp * eY + cfg.finKd * latest.gy);
    float cz = -(cfg.finKp * eZ + cfg.finKd * latest.gz);
    float cr = -(cfg.finRollKp * spinErr + cfg.finRollKd * latest.gx);
    setServo(0, clampf( cy + cr, cfg.finMax));            // fin A
    setServo(1, clampf( cz + cr, cfg.finMax));            // fin B
    setServo(2, clampf(-cy + cr, cfg.finMax));            // fin C
    setServo(3, clampf(-cz + cr, cfg.finMax));            // fin D
  }
}

// Manual mode: each joystick axis drives one servo across its allowed travel.
//  Joystick 1: X -> servo 1, Y -> servo 2.   Joystick 2: X -> servo 3, Y -> servo 4.
void updateServoManual() {
  float j[4] = {joyX, joyY, joy2X, joy2Y};
  if (millis() - manualLastMs > 3000) for (int i = 0; i < 4; i++) j[i] = 0;   // page went quiet -> centre
  for (int i = 0; i < 4; i++) {
    const ServoCfg &c = cfg.sv[modeIdx()][i];
    float ctr = 90 + c.trim;
    float half = fmaxf(0.0f, fminf(c.mx - ctr, ctr - c.mn));
    setServoRaw(i, (c.dir < 0 ? -1 : 1) * j[i] * half);
  }
}

// ---------- Thrust (ESC) ----------
// Output = 0 unless the hardware ARM switch is ON *and* "Thrust enable" is ON.
// MANUAL: web slider (dead-man: zero after 2 s without a page message).
// AUTO:   hover thrust %, optional altitude hold, optional tilt compensation (more
//         thrust when the vehicle leans, to keep the vertical component constant).
void updateThrust(float dt) {
  float vzNew = (latest.altitude_m - lastAltForVz) / dt;
  lastAltForVz = latest.altitude_m;
  vz = 0.9f * vz + 0.1f * vzNew;

  if (!thrAuto && thrEnable && millis() - thrBeatMs > 2000) { thrEnable = false; thrManual = 0; }   // dead-man
  bool allowed = thrEnable && payloadArmed && !(thrAuto && !imuOK);
  float target = 0;
  if (allowed) {
    if (!thrAuto) {
      target = thrManual;
    } else {
      target = cfg.hoverThr;
      if (cfg.thrMethod > 0.5) target += cfg.altKp * (cfg.altTarget - latest.altitude_m) - cfg.altKd * vz;
      if (cfg.tiltComp > 0.5) target /= fmaxf(cosf(latest.tiltY / RAD2DEG) * cosf(latest.tiltZ / RAD2DEG), 0.5f);
      target = fmaxf(target, cfg.thrMin);
    }
    target = constrain(target, 0.0f, cfg.thrMax);
  }
  if (target > thrActual) thrActual = fminf(target, thrActual + cfg.thrRamp * dt);   // ramp up slowly
  else thrActual = target;                                                          // drop at once
  esc.writeMicroseconds(1000 + (int)lroundf(thrActual * 10));
}

// ---------- Setup ----------
void setup() {
  // Indicator / input pins first, so they are in a known state during boot
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(SD_LED_PIN, OUTPUT);
  pinMode(ARMED_LED_PIN, OUTPUT);
  pinMode(SPARE_LED_PIN, OUTPUT);
  digitalWrite(ARMED_LED_PIN, LOW);         // payload SAFE during boot
  pinMode(ARM_IN_PIN, INPUT);               // input-only pin: needs the external 10k pulldown
  pinMode(LOG_STOP_PIN, INPUT_PULLDOWN);    // unconnected = LOW = logging allowed
  setLedPattern(LED_STARTUP);
  setSdLedPattern(SDLED_NOT_LOGGING);       // not logging yet
  updateLed();
  updateSdLed();

  Serial.begin(115200);
  waitMs(500);
  Serial.println("\n--- Flight sensor logger bring-up ---");
  buildParams();
  loadCfg();   // saved panel settings (if any)

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz fast mode

  // BMP280
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("ERROR: BMP280 not found on 0x76 or 0x77. Check wiring.");
  } else {
    Serial.println("BMP280 OK.");
    applyBmp();
  }

  // MPU9250
  initMPU9250();
  calibrateGyro(); // board must be held still for ~2 seconds here (LED blinks slowly)

  // SD card
  setLedPattern(LED_STARTUP);
  stopPinActive = (digitalRead(LOG_STOP_PIN) == HIGH);
  prevWantStop = stopPinActive;
  setupSD();

  // Servos (2 TVC + 2 canards)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  esc.setPeriodHertz(50);
  esc.attach(THRUST_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);   // ESC arms at minimum throttle during the servo tests
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo3.attach(SERVO3_PIN, 500, 2400);
  servo4.attach(SERVO4_PIN, 500, 2400);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  setLedPattern(LED_TVC_TEST);
  runServoStartupSequence();
  setLedPattern(LED_CANARD_TEST);
  runCanardTest();

  // WiFi AP + web server
  setLedPattern(LED_STARTUP);
  setupWiFiServer();

  // Arming is only allowed from here on. If the switch is already ON, it must be
  // cycled OFF then ON.
  armLockout = (digitalRead(ARM_IN_PIN) == HIGH);
  if (armLockout) Serial.println("WARNING: ARM switch is ON at boot - switch it OFF, then ON, to arm.");

  Serial.println(logActive ? "Setup complete. Logging active." : "Setup complete. (NOT logging)");
}

// ---------- Main loop ----------
void loop() {
  server.handleClient(); // must be called often and NOT blocked by delay()
  updateIndicators();    // green solid + SD LED blink rate + ARMED LED

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  // Use the real elapsed time for integration (an SD write can occasionally run long)
  float dt = (now - lastSampleMs) / 1000.0;
  if (dt > 0.25) dt = SAMPLE_INTERVAL_MS / 1000.0; // first pass after boot / long stall
  lastSampleMs = now;

  latest.t = now;
  float bp = bmp.readPressure() / 100.0, ba = bmp.readAltitude(seaLevelHpa), bt = bmp.readTemperature();
  if (!isnan(bp)) latest.pressure_hpa = bp;     // a failed read returns NaN: keep the last good value
  if (!isnan(ba)) latest.altitude_m = ba;
  if (!isnan(bt)) latest.temp_c = bt;

  // --- IMU: only trust data from a read that really succeeded ---
  float rawAx = 0, rawAy = 0, rawAz = 0, rawGx = 0, rawGy = 0, rawGz = 0;
  bool imuRead = imuOK && readMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
  if (imuOK) {
    if (imuRead) imuFailCount = 0;
    else if (++imuFailCount >= 5) { imuOK = false; Serial.println("ERROR: IMU stopped answering. Servos held centred."); }
  } else if (millis() - lastImuRetryMs > 3000) {          // keep trying, so a fixed joint recovers without a reboot
    lastImuRetryMs = millis();
    if (initMPU9250()) { calibrateGyro(); orientInit = false; imuFailCount = 0; Serial.println("IMU recovered."); }
  }

  if (imuOK && imuRead) {
    rawGx -= gyroBiasX;
    rawGy -= gyroBiasY;
    rawGz -= gyroBiasZ;

    // Low-pass (EMA) smoothing against the previous sample to cut jitter
    latest.ax = ACCEL_SMOOTH_ALPHA * rawAx + (1 - ACCEL_SMOOTH_ALPHA) * latest.ax;
    latest.ay = ACCEL_SMOOTH_ALPHA * rawAy + (1 - ACCEL_SMOOTH_ALPHA) * latest.ay;
    latest.az = ACCEL_SMOOTH_ALPHA * rawAz + (1 - ACCEL_SMOOTH_ALPHA) * latest.az;
    latest.gx = GYRO_SMOOTH_ALPHA * rawGx + (1 - GYRO_SMOOTH_ALPHA) * latest.gx;
    latest.gy = GYRO_SMOOTH_ALPHA * rawGy + (1 - GYRO_SMOOTH_ALPHA) * latest.gy;
    latest.gz = GYRO_SMOOTH_ALPHA * rawGz + (1 - GYRO_SMOOTH_ALPHA) * latest.gz;

    readMag(latest.mx, latest.my, latest.mz); // ok if it returns false occasionally
    updateOrientation(latest.ax, latest.ay, latest.az, latest.gx, latest.gy, latest.gz,
                       latest.mx, latest.my, latest.mz, dt);
  }
  if (manualMode) updateServoManual();
  else if (imuOK) updateServosAuto();
  else holdServosCentre();                                  // IMU down: servos held at centre
  if (payloadArmed && !prevArmed) spinRef = latest.spin;    // roll reference = heading when ARM goes on
  prevArmed = payloadArmed;
  updateThrust(dt);

  // Logging stop input: HIGH stops logging (file closed), releasing resumes it
  updateStopPin();
  bool wantStop = stopPinActive || webStop;      // D5 switch OR the web button
  if (wantStop && logActive) stopLogging(stopPinActive ? "stop input HIGH" : "stopped from web panel");
  if (!wantStop && prevWantStop) resumeLogging();
  prevWantStop = wantStop;

  // Print to Serial for live viewing
  Serial.printf("t=%lu  P=%.2fhPa  Alt=%.2fm  T=%.2fC  A=(%.2f,%.2f,%.2f)g  G=(%.1f,%.1f,%.1f)dps  tiltY=%.1f tiltZ=%.1f spin=%.1f  servo=(%d,%d,%d,%d)  thr=%.0f%%  mode=%d  log=%s  armed=%s  imu=%s\n",
                latest.t, latest.pressure_hpa, latest.altitude_m, latest.temp_c,
                latest.ax, latest.ay, latest.az, latest.gx, latest.gy, latest.gz,
                latest.tiltY, latest.tiltZ, latest.spin,
                latest.servo_deg[0], latest.servo_deg[1], latest.servo_deg[2], latest.servo_deg[3],
                thrActual, modeIdx() + 1, logActive ? "ON" : "OFF",
                payloadArmed ? "YES" : "no", imuOK ? "OK" : "FAULT");

  // Append to SD (file stays open; flushed every FLUSH_EVERY_N_SAMPLES)
  if (logActive) {
    size_t written = logFile.printf("%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.1f,%d\n",
                    latest.t, latest.pressure_hpa, latest.altitude_m, latest.temp_c,
                    latest.ax, latest.ay, latest.az, latest.gx, latest.gy, latest.gz,
                    latest.mx, latest.my, latest.mz, latest.tiltY, latest.tiltZ, latest.spin,
                    latest.servo_deg[0], latest.servo_deg[1], latest.servo_deg[2], latest.servo_deg[3],
                    thrActual, cfg.mode < 0.5 ? 1 : 2);
    if (written == 0) {
      Serial.println("WARNING: SD write failed - logging stopped. Card removed or full?");
      logFile.close();
      logActive = false;
      sdMounted = false;   // forces a remount on the next resume
    } else if (++samplesSinceFlush >= FLUSH_EVERY_N_SAMPLES) {
      logFile.flush();
      samplesSinceFlush = 0;
    }
  }
}
