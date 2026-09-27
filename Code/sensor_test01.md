/*
  flight_logger.ino
  ------------------
  BMP280 + MPU9250 read loop, logged to SD as CSV,
  with two servos exercised on their own PWM channels.
  Wiring (ESP32 dev board, 3.3V logic throughout):

    I2C bus (BMP280 + MPU9250, shared):
      SDA -> GPIO 21
      SCL -> GPIO 22
      VCC -> 3.3V   (NOT 5V — both boards are 3.3V native)
      GND -> GND

    SPI bus (SD card module):
      CS   -> GPIO 5
      MOSI -> GPIO 23
      MISO -> GPIO 19
      SCK  -> GPIO 18
      VCC  -> 3.3V or 5V depending on your module (most breakout SD
              modules have an onboard regulator + level shifter — check
              silkscreen; if unsure, 3.3V is always safe)
      GND  -> GND

    Servos (SEPARATE 5V supply, NOT the ESP32 5V pin — except a single
    unloaded servo for bench testing, which can run off VIN/5V briefly):
      Servo 1 signal -> GPIO 4    (drives from roll)
      Servo 2 signal -> GPIO 26   (drives from pitch)
      Servo V+       -> external 5V rail (battery/BEC), or VIN for a quick
                         single-servo unloaded trial
      Servo GND      -> external supply GND, tied to ESP32 GND (common ground)

  Libraries required (install via Arduino Library Manager):
    - Adafruit BMP280 Library   (by Adafruit)
    - Adafruit Unified Sensor   (dependency of the above)
    - ESP32Servo                (by Kevin Harrington / madhephaestus)
    - SD (bundled with ESP32 core)

  MPU9250 is read via direct register access rather than a third-party
  library. This is deliberate: a lot of "MPU-9250" boards on the market
  are actually MPU-6500 + AK8963 clones, and library behavior varies.
  Direct register reads let you see exactly what's on the bus and the
  WHO_AM_I check below will tell you which chip you actually have.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------- WiFi (ESP32 as its own access point) ----------
// Connect your phone to this WiFi network, then browse to 192.168.4.1
const char* AP_SSID = "FlightLogger";
const char* AP_PASSWORD = "flighttest123"; // must be 8+ chars, or set to "" for open network

WebServer server(80);

// ---------- Pin definitions ----------
#define I2C_SDA        21
#define I2C_SCL        22

#define SD_CS_PIN       5   // SPI CS for SD card module (VSPI default MOSI/MISO/SCK used)

#define SERVO1_PIN      4   // moved from 25 -> D4
#define SERVO2_PIN     26

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
Servo servo1, servo2;
File logFile;

const char* LOG_FILENAME = "/flight_log.csv";
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_INTERVAL_MS = 50; // 20 Hz logging

float seaLevelHpa = 1013.25; // recalibrate to local pressure for accurate altitude

// ---------- Latest readings (shared with web server) ----------
// Loop writes these every sample; web handlers just read them, so the
// phone always sees the most recent values without slowing the sample loop.
struct SensorSnapshot {
  unsigned long t = 0;
  float pressure_hpa = 0, altitude_m = 0, temp_c = 0;
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float mx = 0, my = 0, mz = 0;
  float roll = 0, pitch = 0, yaw = 0; // degrees, for the 3D orientation view
} latest;

// ---------- Complementary filter state (for the orientation cube) ----------
float filtRoll = 0, filtPitch = 0, filtYaw = 0;
const float COMP_ALPHA = 0.96; // weight on gyro integration vs accel reference

void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
  // Accelerometer-derived reference angles (only valid when not under heavy
  // linear acceleration -- fine for a bench/hand-tilt test, not mid-flight)
  float accelRoll  = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Gyro integration (deg/s * s = deg), blended with accel reference to stop drift
  filtRoll  = COMP_ALPHA * (filtRoll  + gx * dt) + (1 - COMP_ALPHA) * accelRoll;
  filtPitch = COMP_ALPHA * (filtPitch + gy * dt) + (1 - COMP_ALPHA) * accelPitch;
  filtYaw   += gz * dt; // no absolute reference wired in yet, so this will drift over time

  latest.roll = filtRoll;
  latest.pitch = filtPitch;
  latest.yaw = filtYaw;
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

void mpuReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, (int)count);
  for (int i = 0; i < count && Wire.available(); i++) {
    dest[i] = Wire.read();
  }
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

bool initMPU9250() {
  uint8_t whoami = mpuReadReg(MPU_WHO_AM_I);
  Serial.print("MPU WHO_AM_I = 0x");
  Serial.println(whoami, HEX);
  // Genuine MPU9250 -> 0x71. Some clones/MPU6500-based boards report 0x70 or 0x73.
  if (whoami != 0x71 && whoami != 0x73 && whoami != 0x70) {
    Serial.println("WARNING: unexpected WHO_AM_I. Check wiring/address, or this may be a different chip.");
  }

  mpuWriteReg(MPU_PWR_MGMT_1, 0x00);   // wake up, clock = internal
  delay(100);
  mpuWriteReg(MPU_CONFIG, 0x03);       // DLPF ~44Hz, reduces vibration noise
  mpuWriteReg(MPU_GYRO_CFG, 0x08);     // +-500 dps
  mpuWriteReg(MPU_ACCEL_CFG, 0x08);    // +-4g

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

void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t raw[14];
  mpuReadBytes(MPU_ACCEL_XOUT_H, 14, raw);

  int16_t ax_raw = (raw[0] << 8) | raw[1];
  int16_t ay_raw = (raw[2] << 8) | raw[3];
  int16_t az_raw = (raw[4] << 8) | raw[5];
  // raw[6],raw[7] = temperature, skipped here
  int16_t gx_raw = (raw[8] << 8) | raw[9];
  int16_t gy_raw = (raw[10] << 8) | raw[11];
  int16_t gz_raw = (raw[12] << 8) | raw[13];

  // Scale factors for +-4g accel, +-500dps gyro (match config above)
  ax = ax_raw / 8192.0;   // g
  ay = ay_raw / 8192.0;
  az = az_raw / 8192.0;
  gx = gx_raw / 65.5;     // deg/s
  gy = gy_raw / 65.5;
  gz = gz_raw / 65.5;
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

// ---------- Web server handlers ----------
void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><title>Flight Logger</title>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>"
    "body{font-family:monospace;background:#111;color:#0f0;padding:16px;}"
    "h2{color:#fff;} table{width:100%;border-collapse:collapse;margin-top:20px;}"
    "td{padding:6px 8px;border-bottom:1px solid #333;} "
    "td.label{color:#888;} td.val{text-align:right;}"
    ".scene{width:200px;height:200px;margin:20px auto;perspective:600px;}"
    ".cube{width:100%;height:100%;position:relative;transform-style:preserve-3d;"
    "transition:transform 0.1s linear;}"
    ".face{position:absolute;width:200px;height:200px;border:2px solid #0f0;"
    "background:rgba(0,255,0,0.08);display:flex;align-items:center;justify-content:center;"
    "font-size:14px;color:#0f0;box-sizing:border-box;}"
    ".front {transform: translateZ(100px);}"
    ".back  {transform: translateZ(-100px) rotateY(180deg);}"
    ".right {transform: rotateY(90deg) translateZ(100px);}"
    ".left  {transform: rotateY(-90deg) translateZ(100px);}"
    ".top   {transform: rotateX(90deg) translateZ(100px); background:rgba(255,255,0,0.12); border-color:#ff0;}"
    ".bottom{transform: rotateX(-90deg) translateZ(100px);}"
    "</style></head><body>"
    "<h2>ESP32 Flight Logger — Live</h2>"
    "<div class='scene'><div class='cube' id='cube'>"
    "<div class='face front'>FRONT</div>"
    "<div class='face back'>BACK</div>"
    "<div class='face right'>RIGHT</div>"
    "<div class='face left'>LEFT</div>"
    "<div class='face top'>BOARD UP</div>"
    "<div class='face bottom'>BOTTOM</div>"
    "</div></div>"
    "<table id='t'></table>"
    "<p style='color:#666;font-size:12px;'>Updates every 200ms. Yaw drifts over time (no mag fusion yet) — roll/pitch are stable.</p>"
    "<script>"
    "const cube = document.getElementById('cube');"
    "async function poll(){"
    "  const r = await fetch('/data'); const d = await r.json();"
    "  cube.style.transform = "
    "    `rotateX(${-d.pitch}deg) rotateY(${d.yaw}deg) rotateZ(${d.roll}deg)`;"
    "  const rows = ["
    "    ['roll (deg)', d.roll.toFixed(1)],"
    "    ['pitch (deg)', d.pitch.toFixed(1)],"
    "    ['yaw (deg)', d.yaw.toFixed(1)],"
    "    ['time (ms)', d.t],"
    "    ['pressure (hPa)', d.pressure_hpa.toFixed(2)],"
    "    ['altitude (m)', d.altitude_m.toFixed(2)],"
    "    ['temp (C)', d.temp_c.toFixed(2)],"
    "    ['accel X/Y/Z (g)', d.ax.toFixed(3)+' / '+d.ay.toFixed(3)+' / '+d.az.toFixed(3)],"
    "    ['gyro X/Y/Z (dps)', d.gx.toFixed(1)+' / '+d.gy.toFixed(1)+' / '+d.gz.toFixed(1)],"
    "    ['mag X/Y/Z (uT)', d.mx.toFixed(1)+' / '+d.my.toFixed(1)+' / '+d.mz.toFixed(1)]"
    "  ];"
    "  document.getElementById('t').innerHTML = rows.map(r=>"
    "    `<tr><td class='label'>${r[0]}</td><td class='val'>${r[1]}</td></tr>`).join('');"
    "}"
    "setInterval(poll, 200); poll();"
    "</script></body></html>";
  server.send(200, "text/html", html);
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
  json += "\"roll\":" + String(latest.roll, 1) + ",";
  json += "\"pitch\":" + String(latest.pitch, 1) + ",";
  json += "\"yaw\":" + String(latest.yaw, 1);
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
  server.begin();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Flight sensor logger bring-up ---");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz fast mode

  // BMP280
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("ERROR: BMP280 not found on 0x76 or 0x77. Check wiring.");
  } else {
    Serial.println("BMP280 OK.");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_63);
  }

  // MPU9250
  initMPU9250();

  // SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ERROR: SD card init failed. Check wiring/CS pin/card format (must be FAT32).");
  } else {
    Serial.println("SD card OK.");
    bool needHeader = !SD.exists(LOG_FILENAME);
    logFile = SD.open(LOG_FILENAME, FILE_APPEND);
    if (logFile) {
      if (needHeader) {
        logFile.println("millis,pressure_hpa,altitude_m,temp_c,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT,roll_deg,pitch_deg,yaw_deg");
        logFile.flush();
      }
      logFile.close();
    } else {
      Serial.println("ERROR: could not open log file.");
    }
  }

  // Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo1.write(90);
  servo2.write(90);

  // WiFi AP + web server
  setupWiFiServer();

  Serial.println("Setup complete. Logging at 20 Hz to " + String(LOG_FILENAME));
}

// ---------- Servo driven by orientation ----------
// Servo1 mirrors roll, Servo2 mirrors pitch (both mapped from
// roughly +-90 deg tilt onto the servo's 0-180 deg range).
// Update to whatever mapping matches your actual control-surface geometry
// once you move past this bench test.
void updateServoFromOrientation() {
  int angle1 = constrain((int)(90 + latest.roll), 0, 180);
  int angle2 = constrain((int)(90 + latest.pitch), 0, 180);
  servo1.write(angle1);
  servo2.write(angle2);
}

// ---------- Main loop ----------
void loop() {
  server.handleClient(); // must be called often and NOT blocked by delay()

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  latest.t = now;
  latest.pressure_hpa = bmp.readPressure() / 100.0;
  latest.altitude_m = bmp.readAltitude(seaLevelHpa);
  latest.temp_c = bmp.readTemperature();

  readMPU(latest.ax, latest.ay, latest.az, latest.gx, latest.gy, latest.gz);
  readMag(latest.mx, latest.my, latest.mz); // ok if it returns false occasionally (mag updates slower)
  updateOrientation(latest.ax, latest.ay, latest.az, latest.gx, latest.gy, latest.gz,
                     SAMPLE_INTERVAL_MS / 1000.0);
  updateServoFromOrientation();

  // Print to Serial for live viewing
  Serial.printf("t=%lu  P=%.2fhPa  Alt=%.2fm  T=%.2fC  A=(%.2f,%.2f,%.2f)g  G=(%.1f,%.1f,%.1f)dps  M=(%.1f,%.1f,%.1f)uT\n",
                latest.t, latest.pressure_hpa, latest.altitude_m, latest.temp_c,
                latest.ax, latest.ay, latest.az, latest.gx, latest.gy, latest.gz,
                latest.mx, latest.my, latest.mz);

  // Append to SD
  logFile = SD.open(LOG_FILENAME, FILE_APPEND);
  if (logFile) {
    logFile.printf("%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                    latest.t, latest.pressure_hpa, latest.altitude_m, latest.temp_c,
                    latest.ax, latest.ay, latest.az, latest.gx, latest.gy, latest.gz,
                    latest.mx, latest.my, latest.mz, latest.roll, latest.pitch, latest.yaw);
    logFile.close();
  } else {
    Serial.println("WARNING: could not open log file for append.");
  }
}
