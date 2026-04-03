typedef struct { float x; float y; } Pt2f;
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "html510.h"
#include "vive510.h"

// ===================== Board Note =====================
// This sketch has been re-mapped for ESP32-S3 modules (e.g., ESP32-S3-N8R2).
// Avoid boot/strapping pins GPIO0/GPIO3/GPIO45/GPIO46, and avoid GPIO19/20 if you need USB.
// =======================================================

// ===================== WiFi Settings =====================

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

#define USE_STATIC_IP true
IPAddress local_IP(192,168,1,211);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
IPAddress dns1(8,8,8,8);

HTML510Server server(80);

// ===================== I2C & ToF =====================

#define I2C_SDA   8   // ESP32-S3 recommended I2C SDA
#define I2C_SCL   9   // ESP32-S3 recommended I2C SCL

// ===================== TopHat (I2C) =====================
// TopHat is an I2C SLAVE at address 0x28.
// We (ESP32) are the I2C MASTER. Every 0.5s (2Hz) we send 1 byte =
// number of wireless packets used in the last 0.5s (cap at 255).
// Then we read back 1 byte = health. If health == 0, we must stop motion.

#define TOPHAT_ADDR 0x28
static const uint32_t TOPHAT_PERIOD_MS = 500;     // 2Hz
static const uint32_t TOPHAT_DEATH_LOCK_MS = 15000; // respawn lock (safety)
static const uint8_t  TOPHAT_REPORT_OVERHEAD = 0;      // report only real wireless packet count; 0 means no overhead

volatile uint32_t wirelessPktCounter = 0;

// Call this whenever YOUR code sends/receives a WiFi/BT packet.
// For HTTP control, we call it inside each handler automatically.
inline void COUNT_WIRELESS_PACKET(uint8_t n = 1) { wirelessPktCounter += n; }

// Simple heuristic for a web request: one RX + one TX
inline void COUNT_WIRELESS_REQ() { COUNT_WIRELESS_PACKET(2); }

uint8_t g_tophatHealth = 255;
bool    g_dead         = false;
uint32_t g_deadSinceMs = 0;

bool motionAllowed() {
  // If TopHat reports 0 HP, we must not move.
  if (g_tophatHealth == 0) return false;

  // If we've recently died, enforce a 15s lockout window.
  if (g_dead && (millis() - g_deadSinceMs) < TOPHAT_DEATH_LOCK_MS) return false;

  // Otherwise all motion modes are allowed.
  return true;
}

bool initTopHat() {
  Wire.beginTransmission(TOPHAT_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println("[TopHat] Found at I2C 0x28");
    return true;
  } else {
    Serial.print("[TopHat] NOT found at 0x28, I2C error=");
    Serial.println(err);
    return false;
  }
}

bool tophatSendPacketCount(uint8_t packetCount) {
  Wire.beginTransmission(TOPHAT_ADDR);
  Wire.write(packetCount);
  return (Wire.endTransmission() == 0);
}

bool tophatReadHealth(uint8_t &healthOut) {
  uint8_t n = Wire.requestFrom(TOPHAT_ADDR, (uint8_t)1);
  if (n < 1) return false;
  healthOut = Wire.read();
  return true;
}

// Call this frequently; it will only do work at 2Hz.
void updateTopHat2Hz() {
  static uint32_t lastTickMs = 0;
  static uint32_t lastWirelessCount = 0;

  uint32_t now = millis();
  if (now - lastTickMs < TOPHAT_PERIOD_MS) return;
  lastTickMs += TOPHAT_PERIOD_MS;

  // packets in last 0.5s
  uint32_t cur = wirelessPktCounter;
  uint32_t delta = cur - lastWirelessCount;
  lastWirelessCount = cur;

  // report only real wireless packet count (0..255)
  uint8_t pktByte = (delta > 255) ? 255 : (uint8_t)delta;

  bool okSend = tophatSendPacketCount(pktByte);

  uint8_t h = g_tophatHealth;
  bool okRead = tophatReadHealth(h);
  if (okRead) g_tophatHealth = h;

  Serial.print("[TopHat] pkts=");
  Serial.print(pktByte);
  Serial.print(" send=");
  Serial.print(okSend ? "OK" : "FAIL");
  Serial.print(" read=");
  Serial.print(okRead ? "OK" : "FAIL");
  Serial.print(" health=");
  Serial.println(g_tophatHealth);

  // death handling (TopHat enforces whisker damage/cooldown; we just obey health)
  if (g_tophatHealth == 0 && !g_dead) {
    g_dead = true;
    g_deadSinceMs = now;
    Serial.println("[TopHat] DEAD (health==0). Forcing motor stop + 15s lockout.");
  }

  // If we've been dead and health has recovered AND lock window has passed,
  // clear the dead flag so all motion modes can resume.
  if (g_dead && g_tophatHealth > 0 && (now - g_deadSinceMs) >= TOPHAT_DEATH_LOCK_MS) {
    g_dead = false;
    Serial.println("[TopHat] Respawn complete; motion re-enabled.");
  }
}

#define TOF_FRONT_XSHUT 16   // front ToF XSHUT
#define TOF_RIGHT_XSHUT 17   // right ToF XSHUT
#define TOF_LEFT_XSHUT  18   // left ToF XSHUT (unused, keep defined)

#define TOF_FRONT_ADDR  0x30
#define TOF_RIGHT_ADDR  0x31
#define TOF_LEFT_ADDR   0x32  // unused

Adafruit_VL53L0X tof_front;
Adafruit_VL53L0X tof_right;

// ===================== Motor & L298N =====================

#define ENA_PIN     4   // Left motor PWM (LEDC)
#define IN1_PIN     5
#define IN2_PIN     6

#define ENB_PIN     7   // Right motor PWM (LEDC)
#define IN3_PIN    10
#define IN4_PIN    11

#define PWM_FREQ      15000    // 15 kHz
#define PWM_RES_BITS  10       // 0~1023
#define PWM_MAX_VAL   ((1 << PWM_RES_BITS) - 1)

// ===================== Servo (MG90S) =====================

#define SERVO_PIN          12     // Servo PWM
#define SERVO_FREQ         50     // 50 Hz
#define SERVO_RES_BITS     10
#define SERVO_MAX_DUTY     ((1 << SERVO_RES_BITS) - 1)

// Assume 20 ms period; 0.5~2.5 ms pulse width maps to -90°~+90°
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_PERIOD_US = 20000;  // 1 / 50 Hz

// Servo ATTACK mode parameters
const float SERVO_LIMIT_DEG = 0.0f;    // upper limit angle
const float SERVO_LOW_DEG   = -180.0f; // lower limit angle
const float SERVO_STEP_DEG  = 50.0f;   // step size per update


// ===================== Vive (Localization) =====================
// Uses vive510 library (interrupt-based) on a single signal pin.
// Provides filtered raw Vive coordinates (us time-delta units).
#define VIVE_SIGNAL_PIN 13
Vive510 vive(VIVE_SIGNAL_PIN);

// Vive stability parameters (from vive2test)
const int      VIVE_LOCK_FRAMES        = 8;     // consecutive good frames to "lock"
const uint32_t VIVE_LOST_TIMEOUT_MS    = 800;   // no good frames -> lost
const uint32_t VIVE_RESYNC_COOLDOWN_MS = 500;   // min interval between resync
const int      VIVE_JUMP_THRESH_US     = 1500;  // raw jump reject
const float    VIVE_EMA_ALPHA          = 0.20f; // EMA on raw coords

static inline uint16_t med3_u16(uint16_t a, uint16_t b, uint16_t c) {
  uint16_t m;
  if ((a <= b) && (a <= c)) m = (b <= c) ? b : c;
  else if ((b <= a) && (b <= c)) m = (a <= c) ? a : c;
  else m = (a <= b) ? a : b;
  return m;
}

// Filtered Vive state
bool     g_viveLocked = false;
uint16_t g_viveRawX = 0, g_viveRawY = 0;   // filtered raw (median)
float    g_viveEmaX = 0.0f, g_viveEmaY = 0.0f; // EMA raw
bool     g_viveEmaInit = false;

void initVive() {
  pinMode(VIVE_SIGNAL_PIN, INPUT);
  vive.begin();
  Serial.print("[Vive] Initial sync... status=");
  Serial.println((int)vive.sync(8));
}

void updateVive() {
  // raw history for median filter
  static uint16_t x0=0, x1=0, x2=0;
  static uint16_t y0=0, y1=0, y2=0;
  static int lastGoodX = -1, lastGoodY = -1;

  static int goodStreak = 0;
  static uint32_t lastGoodMs = 0;
  static uint32_t lastResyncMs = 0;

  int st = vive.status();

  if (st == VIVE_RECEIVING) {
    // update history
    x2 = x1; y2 = y1;
    x1 = x0; y1 = y0;
    x0 = vive.xCoord();
    y0 = vive.yCoord();

    uint16_t xr = med3_u16(x0, x1, x2);
    uint16_t yr = med3_u16(y0, y1, y2);

    if (xr == 0 || yr == 0) {
      goodStreak = 0;
    } else {
      if (lastGoodX >= 0 && lastGoodY >= 0) {
        int dx = abs((int)xr - lastGoodX);
        int dy = abs((int)yr - lastGoodY);
        if (dx > VIVE_JUMP_THRESH_US || dy > VIVE_JUMP_THRESH_US) {
          goodStreak = 0;
          goto after_recv;
        }
      }
      lastGoodX = (int)xr;
      lastGoodY = (int)yr;

      g_viveRawX = xr;
      g_viveRawY = yr;
      lastGoodMs = millis();

      goodStreak++;
      if (!g_viveLocked && goodStreak >= VIVE_LOCK_FRAMES) {
        g_viveLocked = true;
        g_viveEmaInit = false;
        Serial.println("[Vive] >>> LOCKED");
      }

      // EMA on raw coords
      if (!g_viveEmaInit) {
        g_viveEmaX = (float)xr;
        g_viveEmaY = (float)yr;
        g_viveEmaInit = true;
      } else {
        g_viveEmaX = (1.0f - VIVE_EMA_ALPHA) * g_viveEmaX + VIVE_EMA_ALPHA * (float)xr;
        g_viveEmaY = (1.0f - VIVE_EMA_ALPHA) * g_viveEmaY + VIVE_EMA_ALPHA * (float)yr;
      }
    }
  } else {
    goodStreak = 0;
  }

after_recv:
  uint32_t now = millis();
  if (now - lastGoodMs > VIVE_LOST_TIMEOUT_MS) {
    if (g_viveLocked) {
      g_viveLocked = false;
      g_viveEmaInit = false;
      Serial.println("[Vive] >>> LOST");
    }
    if (now - lastResyncMs > VIVE_RESYNC_COOLDOWN_MS) {
      lastResyncMs = now;
      Serial.print("[Vive] RESYNC status=");
      Serial.print(st);
      Serial.print(" -> ");
      Serial.println((int)vive.sync(8));
    }
  }
}

// ===================== GoTo Target (Web) =====================
// Web gives TargetX/TargetY in [0,100]. We map into raw Vive coords using bilinear mapping
// based on the 4 measured corner coordinates (Dec 10, 2025).

// Corner points in Vive raw coords
static const Pt2f VIVE_BL = {2920.0f, 6480.0f}; // bottom-left
static const Pt2f VIVE_BR = {2920.0f, 1760.0f}; // bottom-right
static const Pt2f VIVE_TL = {5290.0f, 6480.0f}; // top-left
static const Pt2f VIVE_TR = {5240.0f, 1820.0f}; // top-right

Pt2f mapTargetPercentToVive(float targetX_0_100, float targetY_0_100) {
  float u = constrain(targetX_0_100, 0.0f, 100.0f) / 100.0f; // left->right
  float v = constrain(targetY_0_100, 0.0f, 100.0f) / 100.0f; // bottom->top

  Pt2f out;
  out.x = (1-u)*(1-v)*VIVE_BL.x + u*(1-v)*VIVE_BR.x + (1-u)*v*VIVE_TL.x + u*v*VIVE_TR.x;
  out.y = (1-u)*(1-v)*VIVE_BL.y + u*(1-v)*VIVE_BR.y + (1-u)*v*VIVE_TL.y + u*v*VIVE_TR.y;
  return out;
}

// target as percent + mapped raw Vive coords
float g_targetXPercent = 50.0f;
float g_targetYPercent = 50.0f;
float g_targetViveX = 0.0f;
float g_targetViveY = 0.0f;

bool  g_gotoActive = false;

void updateMappedTarget() {
  Pt2f t = mapTargetPercentToVive(g_targetXPercent, g_targetYPercent);
  g_targetViveX = t.x;
  g_targetViveY = t.y;
}

// GoTo controller parameters (NEW; does not change any existing parameters)
const float GOTO_REACH_THRESH_RAW = 140.0f;   // stop when within this raw distance
const float GOTO_SLOW_THRESH_RAW  = 700.0f;   // start slowing down inside this
const float GOTO_BASE_DUTY        = 0.55f;    // nominal forward duty
const float GOTO_MIN_DUTY         = 0.30f;    // minimum forward duty near goal
const float GOTO_TURN_GAIN        = 0.90f;    // turn gain (scaled by angle error)
const float GOTO_SPIN_THRESH_RAD  = 1.20f;    // if angle error bigger, spin in place
const float GOTO_SPIN_DUTY        = 0.45f;    // duty during spin

void updateGoTo() {
  if (!g_gotoActive) return;

  // Need Vive lock + EMA initialized
  if (!g_viveLocked || !g_viveEmaInit) {
    stopMotors();
    return;
  }

  float px = g_viveEmaX;
  float py = g_viveEmaY;

  float dx = g_targetViveX - px;
  float dy = g_targetViveY - py;
  float dist = sqrtf(dx*dx + dy*dy);

  if (dist < GOTO_REACH_THRESH_RAW) {
    stopMotors();
    return;
  }

  // Estimate heading from recent motion (displacement)
  static bool headingValid = false;
  static float headingRad = 0.0f;
  static bool havePrev = false;
  static float prevx = 0.0f, prevy = 0.0f;

  if (havePrev) {
    float mvx = px - prevx;
    float mvy = py - prevy;
    float speed2 = mvx*mvx + mvy*mvy;
    if (speed2 > 25.0f) {  // moved enough
      headingRad = atan2f(mvy, mvx);
      headingValid = true;
    }
  }
  prevx = px; prevy = py; havePrev = true;

  float desired = atan2f(dy, dx);
  float err = desired - headingRad;
  while (err > PI) err -= 2.0f*PI;
  while (err < -PI) err += 2.0f*PI;

  // Speed scheduling near the goal
  float baseDuty = GOTO_BASE_DUTY;
  if (dist < GOTO_SLOW_THRESH_RAW) {
    float t = dist / GOTO_SLOW_THRESH_RAW; // 0..1
    baseDuty = GOTO_MIN_DUTY + t*(GOTO_BASE_DUTY - GOTO_MIN_DUTY);
  }
  int base = pctToSpeed(baseDuty);

  // If no valid heading yet, just creep forward to establish it
  if (!headingValid) {
    setMotorLeft(base);
    setMotorRight(base);
    return;
  }

  if (fabsf(err) > GOTO_SPIN_THRESH_RAD) {
    int spin = pctToSpeed(GOTO_SPIN_DUTY);
    if (err > 0) { // turn CCW
      setMotorLeft(-spin);
      setMotorRight(spin);
    } else {
      setMotorLeft(spin);
      setMotorRight(-spin);
    }
    return;
  }

  int turn = (int)(GOTO_TURN_GAIN * (err / PI) * (float)PWM_MAX_VAL);
  int l = base - turn;
  int r = base + turn;
  l = constrain(l, -PWM_MAX_VAL, PWM_MAX_VAL);
  r = constrain(r, -PWM_MAX_VAL, PWM_MAX_VAL);

  setMotorLeft(l);
  setMotorRight(r);
}


// ===================== Control Parameters =====================

// ToF wall-detection thresholds (mm)
const int WALL_THRESH_MM = 100;
const int WALLR_THRESH_MM = 75;

// AUTO mode duty (percent)
const float DUTY_FWD_LEFT_SEARCH   = 0.65f;
const float DUTY_FWD_RIGHT_SEARCH  = 0.75f;
const float DUTY_FWD_LEFT_WALL     = 0.75f;
const float DUTY_FWD_RIGHT_WALL    = 0.60f;
const float DUTY_BACK_LEFT_OBS     = 0.75f;
const float DUTY_BACK_RIGHT_OBS    = -0.90f;

// MANUAL mode speeds (percent)
const float MANUAL_FWD_LEFT   = 0.80f;
const float MANUAL_FWD_RIGHT  = 0.80f;
const float MANUAL_BACK_LEFT  = -0.80f;
const float MANUAL_BACK_RIGHT = -0.80f;
const float MANUAL_TURN_L_L   = 0.65f;
const float MANUAL_TURN_L_R   = -0.65f;
const float MANUAL_TURN_R_L   = -0.65f;
const float MANUAL_TURN_R_R   = 0.65f;

// ===================== Mode enums =====================

enum ControlMode {
  MODE_AUTO_WALL = 0,   // Auto right-wall
  MODE_MANUAL    = 1    // Manual (WiFi)
};

enum ServoMode {
  SERVO_HOLD   = 0,     // Hold at a fixed angle
  SERVO_ATTACK = 1      // Sweep back and forth
};

// Added: tower pattern enum (scripted motion)
enum TowerPattern {
  TOWER_NONE = 0,
  TOWER_LOW,
  TOWER_HIGH,
  TOWER_HIGH_RED
};

ControlMode  currentMode       = MODE_AUTO_WALL;
ServoMode    currentServoMode  = SERVO_HOLD;
TowerPattern currentTowerPattern = TOWER_NONE;

// Current servo angle & sweep direction
float servoAngle_deg = 0.0f;
int   servoDir       = 1;  // 1: increasing angle，-1: decreasing angle

// Tower pattern timing
unsigned long towerStartMillis = 0;

// ===================== ToF helper functions =====================

void tofAllShutdown() {
  pinMode(TOF_FRONT_XSHUT, OUTPUT);
  pinMode(TOF_RIGHT_XSHUT, OUTPUT);
  pinMode(TOF_LEFT_XSHUT,  OUTPUT);

  digitalWrite(TOF_FRONT_XSHUT, LOW);
  digitalWrite(TOF_RIGHT_XSHUT, LOW);
  digitalWrite(TOF_LEFT_XSHUT,  LOW);
}

void tofWake(int pin) {
  digitalWrite(pin, HIGH);
  delay(10);
}

bool initToFSensors() {
  tofAllShutdown();

  // FRONT
  Serial.println("Init FRONT ToF ...");
  tofWake(TOF_FRONT_XSHUT);
  if (!tof_front.begin()) {
    Serial.println("ERROR: FRONT ToF begin() failed");
    return false;
  }
  tof_front.setAddress(TOF_FRONT_ADDR);
  Serial.println("  -> FRONT now at 0x30");

  // RIGHT
  Serial.println("Init RIGHT ToF ...");
  tofWake(TOF_RIGHT_XSHUT);
  if (!tof_right.begin()) {
    Serial.println("ERROR: RIGHT ToF begin() failed");
    return false;
  }
  tof_right.setAddress(TOF_RIGHT_ADDR);
  Serial.println("  -> RIGHT now at 0x31");

  return true;
}

bool readToF(Adafruit_VL53L0X &sensor, uint16_t &mm) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  if (measure.RangeStatus == 0) {
    mm = measure.RangeMilliMeter;
    return true;
  } else {
    return false;
  }
}

// ===================== Motor functions =====================

void initMotors() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // New LEDC API
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES_BITS);

  ledcWrite(ENA_PIN, 0);
  ledcWrite(ENB_PIN, 0);

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

// speed in [-PWM_MAX_VAL, PWM_MAX_VAL]
void setMotorLeft(int speed) {
  // TopHat safety: if health==0 (or respawn lock), force no motion
  if (!motionAllowed()) {
    // coast/stop this motor
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, 0);
    return;
  }

  int duty = constrain(abs(speed), 0, PWM_MAX_VAL);

  if (speed > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  ledcWrite(ENA_PIN, duty);
}

void setMotorRight(int speed) {
  // TopHat safety: if health==0 (or respawn lock), force no motion
  if (!motionAllowed()) {
    // coast/stop this motor
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENB_PIN, 0);
    return;
  }

  int duty = constrain(abs(speed), 0, PWM_MAX_VAL);

  if (speed > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else if (speed < 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }
  ledcWrite(ENB_PIN, duty);
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

int pctToSpeed(float pct) {
  return (int)(pct * PWM_MAX_VAL);
}

// ===================== Servo functions =====================

// Angle -> duty
int angleToServoDuty(float angleDeg) {
  // Clamp angle to [-180, 180]
  if (angleDeg > 180.0f)  angleDeg = 180.0f;
  if (angleDeg < -180.0f) angleDeg = -180.0f;

  // Linearly map to pulse width：-180 -> 500us, +180 -> 2500us(simple linear mapping here)
  float pulse_us = 1500.0f + (angleDeg / 180.0f) * 1000.0f;  // approx. 500~2500us

  if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
  if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

  // Convert to duty：pulse / period * maxDuty
  float duty_f = (pulse_us / (float)SERVO_PERIOD_US) * (float)SERVO_MAX_DUTY;
  int duty = (int)(duty_f + 0.5f);
  return duty;
}

void setServoAngle(float angleDeg) {
  int duty = angleToServoDuty(angleDeg);
  ledcWrite(SERVO_PIN, duty);
}

void initServo() {
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES_BITS);
  servoAngle_deg = 50.0f;
  servoDir       = 1;
  setServoAngle(50.0f);   // initial position
}

// Called in loop(): servo behavior depends on currentServoMode
void updateServo() {
  if (currentServoMode == SERVO_ATTACK) {
    // Sweep within [SERVO_LOW_DEG, SERVO_LIMIT_DEG]
    servoAngle_deg += servoDir * SERVO_STEP_DEG;
    if (servoAngle_deg > SERVO_LIMIT_DEG) { 
      servoAngle_deg = SERVO_LIMIT_DEG;
      servoDir = -1;
    } else if (servoAngle_deg < SERVO_LOW_DEG) {
      servoAngle_deg = SERVO_LOW_DEG;
      servoDir = 1;
    }
    setServoAngle(servoAngle_deg);
  }
  // No motion in SERVO_HOLD mode
}

// ===================== Auto right-wall following =====================

void autoStepRightWallSimple() {
  uint16_t dFront = 0, dRight = 0;
  bool okF = readToF(tof_front, dFront);
  bool okR = readToF(tof_right, dRight);

  if (!okF) dFront = 9999;
  if (!okR) dRight = 9999;

  if (dFront <= WALL_THRESH_MM) {
    int leftSpeed  = pctToSpeed(DUTY_BACK_LEFT_OBS);
    int rightSpeed = pctToSpeed(DUTY_BACK_RIGHT_OBS);
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);

    Serial.print("[AUTO] FRONT WALL  | F=");
    Serial.print(dFront);
    Serial.print("mm  R=");
    Serial.print(dRight);
    Serial.print("mm  | Lspd=");
    Serial.print(leftSpeed);
    Serial.print("  Rspd=");
    Serial.println(rightSpeed);
  }
  else if (dRight <= WALLR_THRESH_MM) {
    int leftSpeed  = pctToSpeed(DUTY_FWD_LEFT_WALL);
    int rightSpeed = pctToSpeed(DUTY_FWD_RIGHT_WALL);
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);

    Serial.print("[AUTO] RIGHT WALL | F=");
    Serial.print(dFront);
    Serial.print("mm  R=");
    Serial.print(dRight);
    Serial.print("mm  | Lspd=");
    Serial.print(leftSpeed);
    Serial.print("  Rspd=");
    Serial.println(rightSpeed);
  }
  else {
    int leftSpeed  = pctToSpeed(DUTY_FWD_LEFT_SEARCH);
    int rightSpeed = pctToSpeed(DUTY_FWD_RIGHT_SEARCH);
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);

    Serial.print("[AUTO] SEARCH     | F=");
    Serial.print(dFront);
    Serial.print("mm  R=");
    Serial.print(dRight);
    Serial.print("mm  | Lspd=");
    Serial.print(leftSpeed);
    Serial.print("  Rspd=");
    Serial.println(rightSpeed);
  }
}

// ===================== Tower pattern state machine (non-blocking) =====================

// On each loop(), choose motor outputs based on elapsed time
void updateTowerMotion() {
  if (currentTowerPattern == TOWER_NONE) {
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - towerStartMillis;

  if (currentTowerPattern == TOWER_LOW) {
    // Low Tower:
    // 1) 5s, L=R=0.8
    // 2) 1s, L=0.6, R=0.9
    // 3) 6s, L=R=0.85
    // Then stop

    const unsigned long T1 = 5000;
    const unsigned long T2 = T1 + 1200;
    const unsigned long T3 = T2 + 2000;

    if (elapsed < T1) {
      setMotorLeft(pctToSpeed(0.80f));
      setMotorRight(pctToSpeed(0.80f));
    } else if (elapsed < T2) {
      setMotorLeft(pctToSpeed(0.60f));
      setMotorRight(pctToSpeed(0.90f));
    } else if (elapsed < T3) {
      setMotorLeft(pctToSpeed(0.85f));
      setMotorRight(pctToSpeed(0.85f));
    } else {
      stopMotors();
      currentTowerPattern = TOWER_NONE;
      Serial.println("[TOWER] Low Tower sequence done.");
    }

  } else if (currentTowerPattern == TOWER_HIGH) {
    // High Tower:
    // 1) 8s, L=R=0.8
    // 2) 0.5s, L=0.75, R=0
    // 3) 10s, L=R=0.85
    // Then stop

    const unsigned long T1 = 5400;
    const unsigned long T2 = T1 + 900;
    const unsigned long T3 = T2 + 2000;

    if (elapsed < T1) {
      setMotorLeft(pctToSpeed(0.80f));
      setMotorRight(pctToSpeed(0.80f));
    } else if (elapsed < T2) {
      setMotorLeft(pctToSpeed(0.0f));
      setMotorRight(pctToSpeed(0.75f));  
    } else if (elapsed < T3) {
      setMotorLeft(pctToSpeed(0.85f));
      setMotorRight(pctToSpeed(0.85f));
    } else {
      stopMotors();
      currentTowerPattern = TOWER_NONE;
      Serial.println("[TOWER] High Tower sequence done.");
    }


} else if (currentTowerPattern == TOWER_HIGH_RED) {
  // High Tower (Red):
  // Same timing/parameters as High Tower, except stage-2 motor swap:
  // 1) 8s, L=R=0.8
  // 2) 0.5s, L=0.75, R=0
  // 3) 10s, L=R=0.85
  // Then stop

  const unsigned long T1 = 5400;
  const unsigned long T2 = T1 + 900;
  const unsigned long T3 = T2 + 2000;

  if (elapsed < T1) {
    setMotorLeft(pctToSpeed(0.80f));
    setMotorRight(pctToSpeed(0.80f));
  } else if (elapsed < T2) {
    setMotorLeft(pctToSpeed(0.75f));
    setMotorRight(pctToSpeed(0.0f));
  } else if (elapsed < T3) {
    setMotorLeft(pctToSpeed(0.85f));
    setMotorRight(pctToSpeed(0.85f));
  } else {
    stopMotors();
    currentTowerPattern = TOWER_NONE;
    Serial.println("[TOWER] High Tower (Red) sequence done.");
  }

}

}

// ===================== Web Page & Handlers =====================

void handleRoot() {
  COUNT_WIRELESS_REQ();
  String html = R"=====(<html>
  <head>
    <title>ESP32 Wall Follower</title>
    <meta charset="utf-8">
    <style>
      body { font-family: Arial; text-align: center; margin-top: 20px; }
      .mode-btn { width: 180px; height: 50px; margin: 10px; font-size: 18px; }
      .arrow-btn { width: 80px; height: 60px; font-size: 24px; }
      .servo-btn { width: 120px; height: 40px; margin: 5px; font-size: 16px; }
      .grid { display: inline-block; }
    </style>
    <script>
      function sendCmd(cmd) {
        fetch(cmd).then(r => console.log("Command:", cmd));
      }
    function setTX(v){ document.getElementById("txv").innerText=v; fetch("/set_tx="+v); }
function setTY(v){ document.getElementById("tyv").innerText=v; fetch("/set_ty="+v); }
</script>
  </head>
  <body>
    <h1>ESP32 Wall Follower</h1>

    <p>Mode select:</p>
    <button class="mode-btn" onclick="sendCmd('/mode_auto')">
      Auto Wall Follow
    </button>
    <button class="mode-btn" onclick="sendCmd('/mode_manual')">
      Manual WiFi Control
    </button>

    <hr/>
    <h2>Manual Motion Control</h2>
    <div class="grid">
      <div>
        <button class="arrow-btn" onclick="sendCmd('/cmd_up')">&#9650;</button>
      </div>
      <div>
        <button class="arrow-btn" onclick="sendCmd('/cmd_left')">&#9664;</button>
        <button class="arrow-btn" onclick="sendCmd('/cmd_stop')">■</button>
        <button class="arrow-btn" onclick="sendCmd('/cmd_right')">&#9654;</button>
      </div>
      <div>
        <button class="arrow-btn" onclick="sendCmd('/cmd_down')">&#9660;</button>
      </div>
    </div>

    <hr/>
    <h2>Servo Control</h2>
    <button class="servo-btn" onclick="sendCmd('/cmd_attack')">Attack</button>
    <button class="servo-btn" onclick="sendCmd('/cmd_hold')">Hold</button>

    <hr/>
    <h2>Tower Modes</h2>
    <button class="servo-btn" onclick="sendCmd('/cmd_lowtower')">Low Tower</button>
    <button class="servo-btn" onclick="sendCmd('/cmd_hightower')">High Tower</button>
    <button class="servo-btn" onclick="sendCmd('/cmd_hightower_red')">High Tower (Red)</button>

    <hr/>
    <h2>Vive GoTo Target</h2>
    <p>
      Target X (0~100):
      <input type="range" min="0" max="100" value="50" id="tx" oninput="setTX(this.value)">
      <span id="txv">50</span>
    </p>
    <p>
      Target Y (0~100):
      <input type="range" min="0" max="100" value="50" id="ty" oninput="setTY(this.value)">
      <span id="tyv">50</span>
    </p>
    <button class="mode-btn" onclick="sendCmd('/goto_on')">Start GoTo</button>
    <button class="mode-btn" onclick="sendCmd('/goto_off')">Cancel GoTo</button>

  </body>
</html>)=====";

  server.sendhtml(html);
}

// Mode switch
void handleModeAuto() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_AUTO_WALL;
  currentTowerPattern = TOWER_NONE;  // Cancel tower pattern when switching modes
  stopMotors();
  Serial.println("[WEB] Switch to AUTO WALL FOLLOW mode");
  server.sendplain("Mode AUTO");
}

void handleModeManual() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  currentTowerPattern = TOWER_NONE;
  stopMotors();
  Serial.println("[WEB] Switch to MANUAL WIFI CONTROL mode");
  server.sendplain("Mode MANUAL");
}

// Manual motion commands (any manual command cancels tower pattern)
void handleCmdUp() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  currentTowerPattern = TOWER_NONE;
  int leftSpeed  = pctToSpeed(MANUAL_FWD_LEFT);
  int rightSpeed = pctToSpeed(MANUAL_FWD_RIGHT);
  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);
  Serial.println("[WEB] MANUAL: FORWARD");
  server.sendplain("Forward");
}

void handleCmdDown() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  currentTowerPattern = TOWER_NONE;
  int leftSpeed  = pctToSpeed(MANUAL_BACK_LEFT);
  int rightSpeed = pctToSpeed(MANUAL_BACK_RIGHT);
  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);
  Serial.println("[WEB] MANUAL: BACKWARD");
  server.sendplain("Backward");
}

void handleCmdLeft() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  currentTowerPattern = TOWER_NONE;
  int leftSpeed  = pctToSpeed(MANUAL_TURN_L_L);
  int rightSpeed = pctToSpeed(MANUAL_TURN_L_R);
  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);
  Serial.println("[WEB] MANUAL: TURN LEFT");
  server.sendplain("Turn left");
}

void handleCmdRight() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  currentTowerPattern = TOWER_NONE;
  int leftSpeed  = pctToSpeed(MANUAL_TURN_R_L);
  int rightSpeed = pctToSpeed(MANUAL_TURN_R_R);
  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);
  Serial.println("[WEB] MANUAL: TURN RIGHT");
  server.sendplain("Turn right");
}

void handleCmdStop() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  currentTowerPattern = TOWER_NONE;
  stopMotors();
  Serial.println("[WEB] MANUAL: STOP");
  server.sendplain("Stop");
}

// Servo ATTACK / HOLD commands
void handleCmdAttack() {
  COUNT_WIRELESS_REQ();
  currentServoMode = SERVO_ATTACK;
  Serial.println("[WEB] SERVO: ATTACK mode (sweep)");
  server.sendplain("Servo ATTACK");
}

void handleCmdHold() {
  COUNT_WIRELESS_REQ();
  currentServoMode = SERVO_HOLD;
  servoAngle_deg = 30.0f;
  servoDir       = 1;
  setServoAngle(30.0f);
  Serial.println("[WEB] SERVO: HOLD at 30 deg");
  server.sendplain("Servo HOLD");
}

// Added: Low Tower / High Tower handlers
void handleCmdLowTower() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;          // Treat tower patterns as manual mode
  stopMotors();
  towerStartMillis = millis();
  currentTowerPattern = TOWER_LOW;
  Serial.println("[WEB] TOWER: Low Tower sequence start");
  server.sendplain("Low Tower");
}

void handleCmdHighTower() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  stopMotors();
  towerStartMillis = millis();
  currentTowerPattern = TOWER_HIGH;
  Serial.println("[WEB] TOWER: High Tower sequence start");
  server.sendplain("High Tower");
}

void handleCmdHighTowerRed() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  currentMode = MODE_MANUAL;
  stopMotors();
  towerStartMillis = millis();
  currentTowerPattern = TOWER_HIGH_RED;
  Serial.println("[WEB] TOWER: High Tower (Red) sequence start");
  server.sendplain("High Tower (Red)");
}


// ===================== Vive GoTo Handlers =====================

void handleSetTargetX() {
  COUNT_WIRELESS_REQ();
  int v = server.getVal();
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  g_targetXPercent = (float)v;
  updateMappedTarget();
  Serial.print("[WEB] GOTO: set TargetX=");
  Serial.println(v);
  server.sendplain("TX set");
}

void handleSetTargetY() {
  COUNT_WIRELESS_REQ();
  int v = server.getVal();
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  g_targetYPercent = (float)v;
  updateMappedTarget();
  Serial.print("[WEB] GOTO: set TargetY=");
  Serial.println(v);
  server.sendplain("TY set");
}

void handleGoToOn() {
  COUNT_WIRELESS_REQ();
  currentMode = MODE_MANUAL;           // do not run auto wall follow during goto
  currentTowerPattern = TOWER_NONE;    // cancel tower
  g_gotoActive = true;
  updateMappedTarget();
  Serial.println("[WEB] GOTO: START");
  server.sendplain("GoTo ON");
}

void handleGoToOff() {
  COUNT_WIRELESS_REQ();
  g_gotoActive = false;
  stopMotors();
  Serial.println("[WEB] GOTO: CANCEL");
  server.sendplain("GoTo OFF");
}

// ===================== WiFi & setup/loop =====================


void initWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);

  if (USE_STATIC_IP) {
    if (!WiFi.config(local_IP, gateway, subnet, dns1)) {
      Serial.println("STA Failed to configure");
    }
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin( 5200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32-S3 | ToF Auto Wall Follow + Web Manual + Servo + Tower ===");

  Wire.begin(I2C_SDA, I2C_SCL, 40000);
  initTopHat();
  if (!initToFSensors()) {
    Serial.println("ToF init FAILED, check wiring.");
    while (1) {
      stopMotors();
      delay(1000);
    }
  }

  initMotors();
  initServo();
  initVive();
  updateMappedTarget();
  initWiFi();
  server.begin(80);

  // Register URL handlers
  server.attachHandler("/",            handleRoot);
  server.attachHandler("/mode_auto",   handleModeAuto);
  server.attachHandler("/mode_manual", handleModeManual);

  server.attachHandler("/cmd_up",      handleCmdUp);
  server.attachHandler("/cmd_down",    handleCmdDown);
  server.attachHandler("/cmd_left",    handleCmdLeft);
  server.attachHandler("/cmd_right",   handleCmdRight);
  server.attachHandler("/cmd_stop",    handleCmdStop);

  server.attachHandler("/cmd_attack",  handleCmdAttack);
  server.attachHandler("/cmd_hold",    handleCmdHold);

  // Register tower pattern handlers
  server.attachHandler("/cmd_lowtower",  handleCmdLowTower);
  server.attachHandler("/cmd_hightower", handleCmdHighTower);
  server.attachHandler("/cmd_hightower_red", handleCmdHighTowerRed);


// Vive GoTo handlers
server.attachHandler("/set_tx=",    handleSetTargetX);
server.attachHandler("/set_ty=",    handleSetTargetY);
server.attachHandler("/goto_on",    handleGoToOn);
server.attachHandler("/goto_off",   handleGoToOff);

  currentMode        = MODE_AUTO_WALL;
  currentServoMode   = SERVO_HOLD;
  currentTowerPattern = TOWER_NONE;

  Serial.println("Setup complete. Start in AUTO WALL + SERVO HOLD.");
}

void loop() {
  updateTopHat2Hz();
  updateVive();

  if (!motionAllowed()) {
    stopMotors();
    currentTowerPattern = TOWER_NONE;
    currentServoMode = SERVO_HOLD;
  }

  server.serve();  // Handle web requests


// If a tower pattern is running, it takes over motor outputs
if (currentTowerPattern != TOWER_NONE) {
  updateTowerMotion();
} else if (g_gotoActive) {
  // Vive GoTo overrides normal driving (but still respects TopHat motionAllowed() inside motor functions)
  updateGoTo();
} else {
  // Otherwise, control based on current mode
  if (currentMode == MODE_AUTO_WALL) {
    autoStepRightWallSimple();
  }
  // In manual mode, keep the last command
}

  // Servo control (active in both modes)
  updateServo();

  delay(50);   // ~20Hz main loop
}


