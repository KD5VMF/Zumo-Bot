/*
================================================================================
  Zumo Shield "SMART SUMO / EDGE AVOIDER" (NO LINE FOLLOWING)
================================================================================

  Title:
    Zumo Shield Smart Sumo / Edge Avoider (No Line Following)

  About:
    This Arduino sketch turns a Pololu Zumo Shield robot (Arduino Uno/Leonardo)
    into a “smart” autonomous rover designed for sumo / ring use — *without*
    any line-following behavior.

    The robot uses:
      - Motors (drive control)
      - Onboard user button (digital pin 12 on the Zumo Shield)
      - Buzzer (audio feedback)
      - Reflectance sensor array (for detecting a bright edge / boundary)
      - IMU (accelerometer + magnetometer; gyro if your shield has it)

    Core behavior:
      1) Startup calibration
         - Reflectance sensors are calibrated while the robot spins.
           This makes edge detection much more reliable.
         - If an IMU is detected, the compass (magnetometer) is calibrated
           while the robot spins, enabling heading hold.

      2) Runtime logic (NO LINE FOLLOW)
         - Edge detection: if any reflectance sensor becomes "very bright"
           compared to calibrated range, the robot assumes it is seeing a
           boundary (white edge) and performs an escape maneuver:
             back up -> turn -> continue.

         - Wander/drive: otherwise, it drives forward and (if IMU present)
           uses the compass to hold a chosen heading so it doesn't drift.
           Periodically it chooses a new heading so it explores the arena.

         - Bump reaction: the accelerometer is used to detect sudden jolts
           (a "bump" or collision). When a bump is detected, the robot briefly
           performs an aggressive forward burst (like a sumo “attack”).

    Button controls:
      - Short press (tap):
          cycles mode: AUTO -> RING_ONLY -> WANDER_ONLY -> AUTO ...
      - Long press (hold > ~0.7s):
          toggles STOP / RUN

    Modes explained:
      - AUTO:
          Edge detection active + heading-hold wandering.
      - RING_ONLY:
          Edge detection active + more aggressive forward behavior.
      - WANDER_ONLY:
          Heading-hold wandering; ignores edge rules (useful for testing).

  Hardware notes:
    - This sketch targets the "Zumo Shield for Arduino" library:
        https://github.com/pololu/zumo-shield-arduino-library
    - It is NOT for the Zumo 32U4 robot (that uses different headers).

  How to use:
    1) Install Pololu "ZumoShield" library in Arduino IDE.
    2) Upload this sketch to your Uno/Leonardo with Zumo Shield attached.
    3) Put the robot on your arena/ring surface.
    4) Press the Zumo button (D12) once to begin calibration and start.
    5) Use short/long button presses to switch behaviors at runtime.

  Tuning tips (the two most important settings):
    - EDGE_BRIGHT_THRESHOLD:
        If it escapes too often (false edge triggers), increase it.
        If it drives off the edge and doesn't react, decrease it.
    - BUMP_JERK_THRESHOLD:
        If it triggers “bumps” constantly, increase it.
        If it never triggers, decrease it.

================================================================================
*/

#include <Wire.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <ZumoIMU.h>
#include <ZumoReflectanceSensorArray.h>

// -----------------------------------------------------------------------------
// User button pin
// -----------------------------------------------------------------------------
// On the Zumo Shield for Arduino, the user button is wired to digital pin 12.
// It is typically configured with INPUT_PULLUP, so pressed == LOW internally.
// We use the Pololu Pushbutton helper class, which abstracts this nicely.
#ifndef ZUMO_BUTTON
#define ZUMO_BUTTON 12
#endif

// -----------------------------------------------------------------------------
// Reflectance sensor settings
// -----------------------------------------------------------------------------
// Most Zumo reflectance arrays used with the Zumo Shield have 6 sensors.
// The library supports different counts, but 6 is common and works well here.
static const uint8_t SENSOR_COUNT = 6;
unsigned int sensorValues[SENSOR_COUNT];

// -----------------------------------------------------------------------------
// Motor direction flips (only needed if your wiring is reversed)
// -----------------------------------------------------------------------------
// If your robot drives backward when it should drive forward,
// flip one or both of these flags.
static const bool FLIP_LEFT_MOTOR  = false;
static const bool FLIP_RIGHT_MOTOR = false;

// -----------------------------------------------------------------------------
// Tunable behavior constants
// -----------------------------------------------------------------------------

// Motor speed ranges:
// Pololu Zumo motor driver expects approximately -400..+400.
// 0 = stop, positive = forward, negative = reverse.
static const int16_t MAX_SPEED  = 320;  // “attack” speed
static const int16_t BASE_SPEED = 230;  // normal cruising speed

// Edge detection threshold (calibrated reflectance values are typically ~0..1000).
// Higher = "brighter". Many rings are black center with a white border.
// If any sensor sees a very bright value, we assume we hit the white edge.
static const uint16_t EDGE_BRIGHT_THRESHOLD = 800;

// Bump detection threshold:
// We detect bumps by looking for a sudden change in accelerometer readings
// (a simple "jerk" measure). Raise this if it triggers too easily.
static const int32_t BUMP_JERK_THRESHOLD = 3500;

// Heading hold using magnetometer (compass)
// These values control how strongly we correct heading drift.
static const float HEADING_KP       = 3.0f;  // correction strength
static const float HEADING_DEADBAND = 5.0f;  // ignore tiny heading errors (degrees)

// Timing controls
static const uint32_t WANDER_HEADING_CHANGE_MS = 3500;  // pick new heading interval
static const uint32_t BUMP_REACTION_MS         = 650;   // attack burst duration

// -----------------------------------------------------------------------------
// Hardware objects
// -----------------------------------------------------------------------------
ZumoMotors motors;
ZumoBuzzer buzzer;
Pushbutton button(ZUMO_BUTTON);
ZumoIMU imu;
ZumoReflectanceSensorArray reflectance;

// -----------------------------------------------------------------------------
// Modes
// -----------------------------------------------------------------------------
enum Mode : uint8_t {
  MODE_AUTO = 0,        // edge avoidance + heading-hold wander
  MODE_RING_ONLY = 1,   // edge avoidance + more aggressive forward behavior
  MODE_WANDER_ONLY = 2, // heading-hold wander, ignores edge rules (test mode)
  MODE_STOP = 3         // motors off
};

Mode mode = MODE_AUTO;

// -----------------------------------------------------------------------------
// IMU / Compass state
// -----------------------------------------------------------------------------
bool imuOk = false;  // true if imu.init() succeeded

// Last accelerometer readings used to compute "jerk"
int16_t lastAx = 0, lastAy = 0, lastAz = 0;

// “Bump mode” expiration time: while current time < bumpUntilMs, we do burst.
uint32_t bumpUntilMs = 0;

// Magnetometer calibration min/max values for X/Y.
// Used to normalize magnetometer readings for heading calculation.
int16_t magMinX =  32767, magMinY =  32767;
int16_t magMaxX = -32768, magMaxY = -32768;

// Current heading target (degrees) and when to choose a new target heading
float headingRefDeg = 0.0f;
uint32_t nextHeadingChangeMs = 0;

// -----------------------------------------------------------------------------
// Utility helpers
// -----------------------------------------------------------------------------

// Clamp motor speeds to the safe range expected by the motor driver.
static inline int16_t clampSpeed(int16_t v) {
  if (v >  400) return  400;
  if (v < -400) return -400;
  return v;
}

// Set motor speeds with optional flip flags.
// This is the ONE place where motor direction is corrected.
void setMotorSpeeds(int16_t left, int16_t right) {
  left  = clampSpeed(left);
  right = clampSpeed(right);

  // Flip if needed to correct motor wiring/installation direction.
  if (FLIP_LEFT_MOTOR)  left  = -left;
  if (FLIP_RIGHT_MOTOR) right = -right;

  motors.setSpeeds(left, right);
}

// Keep an angle within [0, 360)
float wrap360(float deg) {
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// Compute shortest signed difference (target - current) in degrees, in [-180, +180]
float angleDiffDeg(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// -----------------------------------------------------------------------------
// Compass / Heading
// -----------------------------------------------------------------------------

// Read heading in degrees using magnetometer X/Y and our calibration.
float readHeadingDeg() {
  if (!imuOk) return 0.0f;

  // Read magnetometer now.
  imu.readMag();

  // Compute center and range for X/Y (simple hard-iron calibration).
  const float cx = (magMinX + magMaxX) * 0.5f;
  const float cy = (magMinY + magMaxY) * 0.5f;
  const float rx = (magMaxX - magMinX) * 0.5f;
  const float ry = (magMaxY - magMinY) * 0.5f;

  // Normalize to roughly [-1..+1] if possible.
  float x = (rx > 1.0f) ? ((imu.m.x - cx) / rx) : (float)imu.m.x;
  float y = (ry > 1.0f) ? ((imu.m.y - cy) / ry) : (float)imu.m.y;

  // Heading angle in degrees.
  float heading = atan2f(y, x) * (180.0f / PI);
  return wrap360(heading);
}

// Calibrate the magnetometer by spinning and capturing min/max values.
// This improves heading accuracy dramatically vs. raw readings.
void calibrateCompass() {
  if (!imuOk) return;

  // Configure IMU for compass heading (library-specific).
  imu.configureForCompassHeading();

  magMinX = magMinY =  32767;
  magMaxX = magMaxY = -32768;

  // Spin for a few seconds to sample all directions.
  uint32_t start = millis();
  while (millis() - start < 4000) {
    setMotorSpeeds(150, -150);  // slow spin

    // Only read mag data when ready if supported.
    if (imu.magDataReady()) {
      imu.readMag();

      if (imu.m.x < magMinX) magMinX = imu.m.x;
      if (imu.m.x > magMaxX) magMaxX = imu.m.x;
      if (imu.m.y < magMinY) magMinY = imu.m.y;
      if (imu.m.y > magMaxY) magMaxY = imu.m.y;
    }
    delay(15);
  }

  setMotorSpeeds(0, 0);

  // Set initial heading target to the current heading.
  headingRefDeg = readHeadingDeg();
}

// -----------------------------------------------------------------------------
// Reflectance calibration and edge detection
// -----------------------------------------------------------------------------

// Calibrate reflectance sensors by spinning and calling reflectance.calibrate().
// IMPORTANT: During calibration, try to ensure the robot sees BOTH the ring
// surface (black) and the border (white), so the calibration spans the range.
void calibrateReflectance() {
  for (uint8_t i = 0; i < 80; i++) {
    // Alternate spin directions so we sweep a wider area.
    if (i < 20 || i >= 60) setMotorSpeeds(180, -180);
    else                   setMotorSpeeds(-180, 180);

    // Update calibration data in the library.
    reflectance.calibrate();
    delay(20);
  }
  setMotorSpeeds(0, 0);
}

// Read calibrated reflectance sensors and compute average/min/max.
// We mostly care about MAX, since a bright border stands out.
void readReflectanceStats(uint16_t &avg, uint16_t &minV, uint16_t &maxV) {
  reflectance.readCalibrated(sensorValues);

  uint32_t sum = 0;
  minV = 65535;
  maxV = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t v = sensorValues[i];
    sum += v;
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
  }

  avg = (uint16_t)(sum / SENSOR_COUNT);
}

// Edge detection rule:
// If ANY sensor reads very bright, assume we're at the border and must escape.
bool edgeSeen(uint16_t maxV) {
  return (maxV > EDGE_BRIGHT_THRESHOLD);
}

// Escape maneuver: back up, then turn away.
// This is intentionally simple and reliable.
void escapeEdge() {
  // Reverse quickly to get back off the border.
  setMotorSpeeds(-240, -240);
  delay(240);

  // Rotate to change direction.
  setMotorSpeeds(-190, 190);
  delay(280);

  // Stop briefly to stabilize.
  setMotorSpeeds(0, 0);
}

// -----------------------------------------------------------------------------
// Bump detection using accelerometer
// -----------------------------------------------------------------------------

// Detect a bump/collision by measuring sudden changes in accelerometer readings.
// This is a very simple heuristic; it works surprisingly well for "hit" detection.
bool detectBump() {
  if (!imuOk) return false;

  imu.readAcc();

  int16_t ax = imu.a.x;
  int16_t ay = imu.a.y;
  int16_t az = imu.a.z;

  // "Jerk" is the sum of absolute differences since last reading.
  int32_t jerk = abs(ax - lastAx) + abs(ay - lastAy) + abs(az - lastAz);

  // Update last readings.
  lastAx = ax;
  lastAy = ay;
  lastAz = az;

  return (jerk > BUMP_JERK_THRESHOLD);
}

// -----------------------------------------------------------------------------
// Movement / “brain” behaviors
// -----------------------------------------------------------------------------

// Wander step:
// - If we're currently in bump-attack window, drive full speed forward.
// - Otherwise, if IMU exists, hold a compass heading and periodically choose
//   a new heading target to explore.
// - If no IMU, just drive forward with occasional random nudges.
void wanderStep(bool useHeadingHold) {
  // If we recently detected a bump, do a short aggressive burst forward.
  if (millis() < bumpUntilMs) {
    setMotorSpeeds(MAX_SPEED, MAX_SPEED);
    return;
  }

  // If we can't / don't want to use heading hold, do “dumb wander”.
  if (!useHeadingHold || !imuOk) {
    // Every few seconds, do a small randomized nudge turn.
    if (millis() >= nextHeadingChangeMs) {
      int dir = (random(0, 2) == 0) ? -1 : 1; // -1 or +1
      setMotorSpeeds(BASE_SPEED + 60 * dir, BASE_SPEED - 60 * dir);
      delay(160);
      nextHeadingChangeMs = millis() + WANDER_HEADING_CHANGE_MS;
    } else {
      setMotorSpeeds(BASE_SPEED, BASE_SPEED);
    }
    return;
  }

  // Heading-hold wander:
  // Occasionally choose a new heading target.
  if (millis() >= nextHeadingChangeMs) {
    headingRefDeg = readHeadingDeg();
    long r = random(-45, 46);                 // random offset in degrees
    headingRefDeg = wrap360(headingRefDeg + (float)r);
    nextHeadingChangeMs = millis() + WANDER_HEADING_CHANGE_MS;
  }

  // Compute heading error and correct motor speeds.
  float current = readHeadingDeg();
  float d = angleDiffDeg(headingRefDeg, current);

  int16_t left = BASE_SPEED;
  int16_t right = BASE_SPEED;

  // Only correct if error is outside a small deadband.
  if (fabs(d) > HEADING_DEADBAND) {
    int16_t corr = (int16_t)constrain(d * HEADING_KP, -90.0f, 90.0f);
    left  += corr;
    right -= corr;
  }

  setMotorSpeeds(left, right);
}

// -----------------------------------------------------------------------------
// Button UI (short press cycles modes, long press toggles stop/run)
// -----------------------------------------------------------------------------

// Small audio “tune” per mode so you can tell what mode you are in.
void playModeTune(Mode m) {
  switch (m) {
    case MODE_AUTO:        buzzer.play(">c16g16"); break;
    case MODE_RING_ONLY:   buzzer.play(">g16e16"); break;
    case MODE_WANDER_ONLY: buzzer.play(">e16c16"); break;
    case MODE_STOP:        buzzer.play("l8 c");    break;
  }
  delay(120);
}

// Reads button press length and updates mode.
// - Tap: cycle modes (if not stopped).
// - Hold: toggle STOP / RUN.
void handleButton() {
  static uint32_t pressStart = 0;
  static bool wasPressed = false;

  bool pressed = button.isPressed();

  // Transition: not pressed -> pressed
  if (pressed && !wasPressed) {
    pressStart = millis();
  }

  // Transition: pressed -> not pressed (release)
  if (!pressed && wasPressed) {
    uint32_t held = millis() - pressStart;

    // Long press: toggle STOP/RUN
    if (held > 700) {
      mode = (mode == MODE_STOP) ? MODE_AUTO : MODE_STOP;
      playModeTune(mode);
    }
    // Short press: cycle mode (only if running)
    else {
      if (mode != MODE_STOP) {
        mode = (Mode)((mode + 1) % 3); // AUTO -> RING_ONLY -> WANDER_ONLY -> AUTO
        playModeTune(mode);
      }
    }
  }

  wasPressed = pressed;
}

// -----------------------------------------------------------------------------
// Arduino setup() and loop()
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // Seed randomness for wander behavior.
  randomSeed(analogRead(A0));

  // Button is usually wired active-low, so use pull-up.
  pinMode(ZUMO_BUTTON, INPUT_PULLUP);

  // Quick startup sound.
  buzzer.play("l16 cdegr");
  delay(250);

  // Initialize I2C (IMU uses I2C).
  Wire.begin();

  // Try to initialize the IMU. If it fails, we still run without IMU features.
  imuOk = imu.init();
  if (imuOk) {
    imu.enableDefault();
  }

  // Initialize reflectance sensors.
  reflectance.init();

  // User instructions
  Serial.println(F("\n========================================"));
  Serial.println(F(" Zumo Smart Sumo / Edge Avoider"));
  Serial.println(F(" (NO line following)"));
  Serial.println(F("========================================"));
  Serial.println(F("Press button to calibrate + start."));
  Serial.println(F("Short press: AUTO / RING_ONLY / WANDER_ONLY"));
  Serial.println(F("Long press: STOP / RUN"));
  Serial.println();

  // Wait for user to begin calibration/start.
  button.waitForButton();

  // Reflectance calibration
  Serial.println(F("Calibrating reflectance (spin/sweep)..."));
  calibrateReflectance();

  // Compass calibration (if IMU exists)
  if (imuOk) {
    Serial.println(F("Calibrating compass (spin)..."));
    calibrateCompass();
    Serial.print(F("Heading ref = "));
    Serial.println(headingRefDeg);

    // Prime accelerometer history for bump detection.
    imu.readAcc();
    lastAx = imu.a.x;
    lastAy = imu.a.y;
    lastAz = imu.a.z;
  } else {
    Serial.println(F("IMU not detected; running without IMU features."));
  }

  // Start wander timing
  nextHeadingChangeMs = millis() + 800;

  // Ready tone
  buzzer.play(">g8");
}

void loop() {
  // Read UI button
  handleButton();

  // If stopped, ensure motors are off and do nothing else.
  if (mode == MODE_STOP) {
    setMotorSpeeds(0, 0);
    delay(10);
    return;
  }

  // Bump detection (if IMU exists)
  if (imuOk && detectBump()) {
    bumpUntilMs = millis() + BUMP_REACTION_MS;
    buzzer.play("l16 >c"); // short chirp to indicate collision
  }

  // Read reflectance stats
  uint16_t avg, minV, maxV;
  readReflectanceStats(avg, minV, maxV);

  // Edge avoidance is active in AUTO and RING_ONLY.
  // (WANDER_ONLY ignores edge rules for testing.)
  if (mode == MODE_AUTO || mode == MODE_RING_ONLY) {
    if (edgeSeen(maxV)) {
      // We detected a boundary edge -> immediately escape.
      escapeEdge();
      return;
    }
  }

  // Decide movement based on mode.
  switch (mode) {
    case MODE_RING_ONLY:
      // Same wandering function, but in practice this “feels” more aggressive
      // because edge avoidance is active and bump bursts are active.
      wanderStep(true);
      break;

    case MODE_WANDER_ONLY:
      // Wander even if we might run off a border (test mode).
      wanderStep(true);
      break;

    case MODE_AUTO:
    default:
      // Edge avoidance + heading-hold wander (best general behavior).
      wanderStep(true);
      break;
  }

  // Small delay keeps loop stable and reduces I2C hammering.
  delay(10);
}
