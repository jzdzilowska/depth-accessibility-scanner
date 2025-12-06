#include <WDT.h>
#include <Stepper.h>


// ================== CONFIG ==================

// Watchdog: reset if not refreshed within this time (ms)
const long WDT_INTERVAL_MS = 3000; 
unsigned long lastWdtRefresh = 0;


// Logical grid of the field (NOT physical actuators)
const int GRID_SIZE = 5;          // can change to 20 later
const int MAX_HEIGHT_DEG = 60;    // max "height" for tactile pixels - NOT cm!

// ---- STEPPER CONFIG ----
const int STEPS_PER_REV_X = 2048;   // 28BYJ-48
const int STEPS_PER_REV_Y = 2048;

// Stepper pins (X axis: pan)
const int STEPPER_X_IN1 = 8;
const int STEPPER_X_IN2 = 9;
const int STEPPER_X_IN3 = 10;
const int STEPPER_X_IN4 = 11;

// Stepper pins (Y axis: tilt)
const int STEPPER_Y_IN1 = 4;
const int STEPPER_Y_IN2 = 5;
const int STEPPER_Y_IN3 = 6;
const int STEPPER_Y_IN4 = 7;

// Ultrasonic HC-SR04
const int TRIG_PIN  = 12;
const int ECHO_PIN  = 13;

// System button: start / toggle
const int BUTTON_PIN = 2;   // INPUT_PULLUP, active LOW

// Angular range of scanner
const int ANGLE_X_MIN = 30;   // degrees
const int ANGLE_X_MAX = 150;
const int ANGLE_Y_MIN = 30;
const int ANGLE_Y_MAX = 150;

// Distance range (cm)
const float DIST_MIN_CM = 3.0;
const float DIST_MAX_CM = 400.0;

// "Big jump" threshold (cm) → end of object in that row
const float JUMP_THRESHOLD_CM = 20.0;

// Debounce / toggle timing
const unsigned long TOGGLE_DEBOUNCE_MS = 200;

// ================== GLOBALS ==================

// Steppers for pan/tilt
Stepper stepperX(STEPS_PER_REV_X, STEPPER_X_IN1, STEPPER_X_IN3, STEPPER_X_IN2, STEPPER_X_IN4);
Stepper stepperY(STEPS_PER_REV_Y, STEPPER_Y_IN1, STEPPER_Y_IN3, STEPPER_Y_IN2, STEPPER_Y_IN4);

// Track current step positions (absolute, from power-on)
long currentStepsX = 0;
long currentStepsY = 0;

// Steps per degree
const float STEPS_PER_DEG_X = (float)STEPS_PER_REV_X / 360.0f;
const float STEPS_PER_DEG_Y = (float)STEPS_PER_REV_Y / 360.0f;

// Scene data
float distanceGrid[GRID_SIZE][GRID_SIZE];  // cm
int   heightGrid[GRID_SIZE][GRID_SIZE];    // 0..MAX_HEIGHT_DEG

// Global min/max distance for current scan (used by height())
float gMinDist = DIST_MAX_CM;
float gMaxDist = DIST_MIN_CM;

// System status & timing
bool systemStatus = false;                // FALSE = OFF, TRUE = ON
unsigned long savedClock = 0;            // generic timestamp
unsigned long lastToggleTime = 0;        // for button debounce

// ================== FSM ==================

enum SystemState {
  INIT,   // initialize system
  CALC,   // perform scan + compute heights
  WAIT,   // idle, wait for user
  END     // after a scan is done, go back to WAIT
};

SystemState systemState = INIT;

// ================== UTILITY / SPEC-FUNCTIONS ==================

// Read distance from HC-SR04 (in cm)
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  if (duration == 0) {
    return DIST_MAX_CM;  // no echo → treat as "very far"
  }

  float distance = (duration * 0.0343f) / 2.0f;  // us → cm

  if (distance < DIST_MIN_CM) distance = DIST_MIN_CM;
  if (distance > DIST_MAX_CM) distance = DIST_MAX_CM;
  return distance;
}

// -------- FSM utils from your spec --------

// Toggle the system ON/OFF based on button and time
bool systemToggle(bool currentStatus, unsigned long now, unsigned long toggleTime) {
  bool button = (digitalRead(BUTTON_PIN) == LOW); // active LOW

  // edge: HIGH -> LOW (button press)
  static bool lastButton = HIGH;

  if (button && !lastButton && (now - lastToggleTime) >= toggleTime) {
    lastToggleTime = now;
    lastButton = button;
    return !currentStatus;  // flip status
  }

  lastButton = button;
  return currentStatus;
}

// Initialize scene, sensors, motors, speed, etc.
void initializeSystem() {
  // Clear scene arrays
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      distanceGrid[y][x] = DIST_MAX_CM;
      heightGrid[y][x]   = 0;
    }
  }

  // Reset global min/max for heights
  gMinDist = DIST_MAX_CM;
  gMaxDist = DIST_MIN_CM;

  // Reset motor positions
  currentStepsX = 0;
  currentStepsY = 0;

  // Reset timers
  savedClock = millis();

  Serial.println("System initialized (INIT).");
}

// Add a height value into the scene (equivalent to scene[y][x] = height)
void add(unsigned int x, unsigned int y, unsigned int h) {
  if (x >= GRID_SIZE || y >= GRID_SIZE) return;
  heightGrid[y][x] = (int)h;
}

// Convert raw distance reading → height for actuator / tactile pixel
// Uses global gMinDist, gMaxDist from the current scan
int height(unsigned long distance) {
  float d = (float)distance;

  if (d >= DIST_MAX_CM - 0.5f) {
    return 0;  // background
  }
  if (gMaxDist <= gMinDist) {
    return 0;  // degenerate case
  }

  float ratio = (gMaxDist - d) / (gMaxDist - gMinDist); // 0..1
  if (ratio < 0) ratio = 0;
  if (ratio > 1) ratio = 1;

  int h = (int)(ratio * MAX_HEIGHT_DEG + 0.5f); // 0..60
  return h;
}

// Check angle in degrees is within [0,180]
bool outOfBoundsAngle(unsigned long angle) {
  return (angle > 180UL);
}

// Placeholder for linear motion check (not used yet)
bool outOfBoundsLinear(unsigned long position) {
  // Suppose valid range is [0, 100] as in your spec
  return (position > 100UL);
}

// Map grid index → angles
int gridToAngleX(int x) {
  return map(x, 0, GRID_SIZE - 1, ANGLE_X_MIN, ANGLE_X_MAX);
}

int gridToAngleY(int y) {
  return map(y, 0, GRID_SIZE - 1, ANGLE_Y_MIN, ANGLE_Y_MAX);
}

// Calculate step delta needed for next pixel (conceptual calcAngleSteps)
long calcAngleSteps(long currentSteps, unsigned int x, unsigned int y, bool isXaxis) {
  int angleDeg = isXaxis ? gridToAngleX(x) : gridToAngleY(y);
  if (outOfBoundsAngle((unsigned long)angleDeg)) {
    return 0; // do nothing if out-of-bounds
  }

  float stepsPerDeg = isXaxis ? STEPS_PER_DEG_X : STEPS_PER_DEG_Y;
  long targetSteps = (long)(angleDeg * stepsPerDeg + 0.5f);
  long deltaSteps = targetSteps - currentSteps;
  return deltaSteps;
}

// Move scanner (sensor) to direction corresponding to (x,y)
void moveScannerTo(int x, int y) {
  long deltaX = calcAngleSteps(currentStepsX, x, y, true);
  long deltaY = calcAngleSteps(currentStepsY, x, y, false);

  if (deltaX != 0) {
    stepperX.step(deltaX);
    currentStepsX += deltaX;
  }
  if (deltaY != 0) {
    stepperY.step(deltaY);
    currentStepsY += deltaY;
  }

  delay(10); // settle
}

// ================== SCANNING LOGIC ==================

// Fill distanceGrid[y][x] with measured distances (cm)
void scanField() {
  Serial.println("=== Starting field scan (CALC) ===");

  for (int y = 0; y < GRID_SIZE; y++) {
    float prevDist = 0.0;
    bool endOfObjectRow = false;

    for (int x = 0; x < GRID_SIZE; x++) {
      WDT.refresh();
      moveScannerTo(x, y);
      float d = readDistanceCm();

      // "Big jump" logic in a row
      if (x > 0) {
        float diff = d - prevDist;
        if (diff < 0) diff = -diff;
        if (diff > JUMP_THRESHOLD_CM) {
          endOfObjectRow = true;
        }
      }

      if (endOfObjectRow) {
        distanceGrid[y][x] = DIST_MAX_CM;
      } else {
        distanceGrid[y][x] = d;
      }

      prevDist = d;

      // Debug print
      Serial.print(distanceGrid[y][x], 1);
      Serial.print(x < GRID_SIZE - 1 ? ", " : "\n");
    }
  }

  Serial.println("=== Field scan complete ===");
}

// Compute gMinDist, gMaxDist and fill heightGrid using height()
void computeHeights() {
  // First pass: find global min/max distance
  gMinDist = DIST_MAX_CM;
  gMaxDist = DIST_MIN_CM;

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      float d = distanceGrid[y][x];
      if (d < gMinDist) gMinDist = d;
      if (d > gMaxDist && d < DIST_MAX_CM) gMaxDist = d;
    }
  }

  if (gMaxDist <= gMinDist) {
    gMaxDist = gMinDist + 1.0f;
  }

  // Second pass: convert each distance to height and store via add()
  Serial.println("=== Height matrix (0..60) ===");
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      int h = height((unsigned long)distanceGrid[y][x]);
      add(x, y, h);  // fills heightGrid[y][x]

      Serial.print(heightGrid[y][x]);
      Serial.print(x < GRID_SIZE - 1 ? ", " : "\n");
    }
  }
  Serial.println("=============================");
}

// Send one frame of distanceGrid for Python / external viewer
void transmitFrame() {
  Serial.println("FRAME_BEGIN");
  Serial.print("GRID_SIZE=");
  Serial.println(GRID_SIZE);
  Serial.println("DISTANCES");

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      Serial.print(distanceGrid[y][x], 2);
      if (x < GRID_SIZE - 1) Serial.print(',');
    }
    Serial.println();
  }

  Serial.println("FRAME_END");
}

// ================== SETUP & LOOP ==================

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  stepperX.setSpeed(10);
  stepperY.setSpeed(10);

  if (WDT_INTERVAL_MS < 1) {
    Serial.println("Invalid watchdog interval");
    while (1) {}   // trap
  }

  if (WDT.begin(WDT_INTERVAL_MS)) {
    Serial.print("WDT started, timeout = ");
    WDT.refresh();
    Serial.print(WDT.getTimeout());   
    WDT.refresh();
    Serial.println(" ms");
    WDT.refresh();
  } else {
    Serial.println("Error initializing watchdog");
    while (1) {}  // trap
  }

  Serial.println("Depth Accessibility Scanner starting...");
  systemState = INIT;
}


void loop() {
  unsigned long now = millis();

  if (now - lastWdtRefresh >= 500) { 
    WDT.refresh();
    lastWdtRefresh = now;
  }

  // High-level FSM
  switch (systemState) {
    case INIT:
      initializeSystem();
      systemStatus = false;      // system starts OFF
      systemState = WAIT;        // go wait for user
      break;

    case WAIT:
      // System is idle; user can toggle ON
      systemStatus = systemToggle(systemStatus, now, TOGGLE_DEBOUNCE_MS);

      if (systemStatus) {
        // toggled ON → start a scan
        Serial.println("System toggled ON → CALC");
        savedClock = now;
        systemState = CALC;
      }
      break;

    case CALC:
      // Perform the scan and compute heights
      scanField();
      computeHeights();
      transmitFrame(); // optional: send data out

      // After one full scan, turn system OFF and go to END
      systemStatus = false;
      Serial.println("Scan complete → system OFF, going to END");
      systemState = END;
      break;

    case END:
      // End of one full cycle; could do cleanup or homing here.
      // For now, just go back to WAIT and allow another toggle.
      systemState = WAIT;
      break;
  }
}
