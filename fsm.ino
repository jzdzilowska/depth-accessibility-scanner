#include <Stepper.h>

// ================== CONFIG ==================

// Grid: 20x20 "pixels" of the *field* (not physical motor positions)
// 5 for testing; change to 20 for final
const int GRID_SIZE = 5;

// Heights for tactile pixels (normalized scale. This ISN'T mm!)
const int MAX_HEIGHT_DEG = 60;

// ---- STEPPER CONFIG ----
// Steps per revolution (will need to change this for our motor)
const int STEPS_PER_REV_X = 2048; 
const int STEPS_PER_REV_Y = 2048;

// Stepper pins
const int STEPPER_X_IN1 = 8;
const int STEPPER_X_IN2 = 9;
const int STEPPER_X_IN3 = 10;
const int STEPPER_X_IN4 = 11;

const int STEPPER_Y_IN1 = 4;
const int STEPPER_Y_IN2 = 5;
const int STEPPER_Y_IN3 = 6;
const int STEPPER_Y_IN4 = 7;

// Ultrasonic HC-SR04 pins
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// Button to start scanning
const int BUTTON_PIN = 2;    // active LOW

// Angular range of the scanner
const int ANGLE_X_MIN = 30;   // left. I.e., gridx=0, then angle=30
const int ANGLE_X_MAX = 150;  // right. I.e., gridx=19, then angle=150
const int ANGLE_Y_MIN = 30;   // up
const int ANGLE_Y_MAX = 150;  // down

// Distance range (cm)
const float DIST_MIN_CM = 3.0;    // ignore unrealistically close readings
const float DIST_MAX_CM = 400.0;  // treat beyond this as "nothing there"

// "Big jump in distance" threshold (cm)
const float JUMP_THRESHOLD_CM = 20.0;

// ================== GLOBALS ==================

// Steppers for pan/tilt
Stepper stepperX(STEPS_PER_REV_X, STEPPER_X_IN1, STEPPER_X_IN3, STEPPER_X_IN2, STEPPER_X_IN4);
Stepper stepperY(STEPS_PER_REV_Y, STEPPER_Y_IN1, STEPPER_Y_IN3, STEPPER_Y_IN2, STEPPER_Y_IN4);

// Track current step positions (we assume start at 0 at reset)
long currentStepsX = 0;
long currentStepsY = 0;

// Precomputed steps per degree
const float STEPS_PER_DEG_X = (float)STEPS_PER_REV_X / 360.0f;
const float STEPS_PER_DEG_Y = (float)STEPS_PER_REV_Y / 360.0f;

// Raw distance matrix (field depth)
float distanceGrid[GRID_SIZE][GRID_SIZE];  // in cm

// Height matrix for tactile grid (0..MAX_HEIGHT_DEG)
int heightGrid[GRID_SIZE][GRID_SIZE];      // degrees or level

enum SystemState {
  IDLE,
  SCANNING,
  READY
};

SystemState systemState = IDLE;

// ================== UTILITY FUNCTIONS ==================

// Read distance from HC-SR04 (in cm)
float readDistanceCm() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  if (duration == 0) {
    // No echo → treat as "very far"
    return DIST_MAX_CM;
  }

  // Speed of sound ~343 m/s → 0.0343 cm/us
  float distance = (duration * 0.0343f) / 2.0f;

  // Clamp to range
  if (distance < DIST_MIN_CM) distance = DIST_MIN_CM;
  if (distance > DIST_MAX_CM) distance = DIST_MAX_CM;

  return distance;
}

// Map grid index [0..GRID_SIZE-1] → servo-equivalent angle
int gridToAngleX(int x) {
  return map(x, 0, GRID_SIZE - 1, ANGLE_X_MIN, ANGLE_X_MAX);
}

int gridToAngleY(int y) {
  return map(y, 0, GRID_SIZE - 1, ANGLE_Y_MIN, ANGLE_Y_MAX);
}

// Convert a target angle to a target step count
long angleToStepsX(int angleDeg) {
  return (long)(angleDeg * STEPS_PER_DEG_X + 0.5f);
}

long angleToStepsY(int angleDeg) {
  return (long)(angleDeg * STEPS_PER_DEG_Y + 0.5f);
}

// Move scanner (sensor) to direction corresponding to (x,y) grid index
void moveScannerTo(int x, int y) {
  int angleX = gridToAngleX(x);
  int angleY = gridToAngleY(y);

  long targetStepsX = angleToStepsX(angleX);
  long targetStepsY = angleToStepsY(angleY);

  long deltaX = targetStepsX - currentStepsX;
  long deltaY = targetStepsY - currentStepsY;

  // Move X axis
  if (deltaX != 0) {
    stepperX.step(deltaX);
    currentStepsX = targetStepsX;
  }

  // Move Y axis
  if (deltaY != 0) {
    stepperY.step(deltaY);
    currentStepsY = targetStepsY;
  }

  // small pause to let vibrations settle
  delay(10);
}

// ================== SCANNING LOGIC ==================

// This fills distanceGrid[y][x] with measured distances (cm)
void scanField() {
  Serial.println("=== Starting field scan ===");

  for (int y = 0; y < GRID_SIZE; y++) {
    float prevDist = 0.0;
    bool endOfObject = false;

    for (int x = 0; x < GRID_SIZE; x++) {
      // Rotate scanner to direction for this (x,y)
      moveScannerTo(x, y);
      float d = readDistanceCm();

      // "Big jump" logic: if current reading is far from previous → end of object
      if (x > 0) {
        float diff = d - prevDist;
        if (diff < 0) diff = -diff;  // absolute value

        if (diff > JUMP_THRESHOLD_CM) {
          endOfObject = true;
        }
      }

      if (endOfObject) {
        // Everything after big jump in this row is treated as background
        distanceGrid[y][x] = DIST_MAX_CM;
      } else {
        distanceGrid[y][x] = d;
      }

      prevDist = d;
      Serial.print(distanceGrid[y][x], 1);
      Serial.print(x < GRID_SIZE - 1 ? ", " : "\n");
    }
  }

  Serial.println("=== Field scan complete ===");
}

// Convert distanceGrid → heightGrid (0..MAX_HEIGHT_DEG)
void computeHeights() {
  // 1) Find the closest distance in the whole field (highest object)
  float minDist = DIST_MAX_CM;
  float maxDist = DIST_MIN_CM;

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      float d = distanceGrid[y][x];
      if (d < minDist) minDist = d;
      if (d > maxDist && d < DIST_MAX_CM) maxDist = d; // ignore "background"
    }
  }

  // Prevent divide-by-zero if everything is essentially the same
  if (maxDist <= minDist) {
    maxDist = minDist + 1.0f;
  }

  // 2) Map distances to heights:
  //    - minDist → MAX_HEIGHT_DEG (highest pixel)
  //    - maxDist → 0
  //    - background (DIST_MAX_CM) → 0
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      float d = distanceGrid[y][x];

      if (d >= DIST_MAX_CM - 0.5f) {
        // Background / no object
        heightGrid[y][x] = 0;
      } else {
        float ratio = (maxDist - d) / (maxDist - minDist); // 0..1
        if (ratio < 0) ratio = 0;
        if (ratio > 1) ratio = 1;

        int h = (int)(ratio * MAX_HEIGHT_DEG + 0.5f); // 0..60 (approx)
        heightGrid[y][x] = h;
      }
    }
  }

  Serial.println("=== Height matrix (0..60) ===");
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      Serial.print(heightGrid[y][x]);
      Serial.print(x < GRID_SIZE - 1 ? ", " : "\n");
    }
  }
  Serial.println("=============================");
}

// Send one "frame" of distance data in a clean, parseable format
void transmitFrame() {
  Serial.println("FRAME_BEGIN");
  Serial.print("GRID_SIZE=");
  Serial.println(GRID_SIZE);
  Serial.println("DISTANCES");

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      Serial.print(distanceGrid[y][x], 2); // cm, 2 decimal places
      if (x < GRID_SIZE - 1) {
        Serial.print(',');
      }
    }
    Serial.println();
  }

  Serial.println("FRAME_END");
}


// ================== SETUP & LOOP ==================

void setup() {
  Serial.begin(115200); // set baud rate
  delay(2000);

  Serial.println("setup start"); 

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // button to GND when pressed

  stepperX.setSpeed(10);  // 10 RPM example
  stepperY.setSpeed(10);

  currentStepsX = 0;
  currentStepsY = 0;

  Serial.println("Depth scanner (stepper) ready. Press button to scan.");
}

void loop() {
  delay(200); 
  int buttonState = digitalRead(BUTTON_PIN);

  switch (systemState) {
    case IDLE:
      // Wait for button press to start scanning
      if (buttonState == LOW) {
        delay(50); // simple debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
          systemState = SCANNING;
          Serial.println("Button pressed → starting scan.");
        }
      }
      break;

    case SCANNING:
      scanField();        // fill distanceGrid
      computeHeights();   // fill heightGrid (0..60)
      transmitFrame();    // send distanceGrid as a "frame" for Python
      systemState = READY;
      Serial.println("Scan + height computation complete. READY.");
      break;

    case READY:
      // At this point:
      //  - distanceGrid[y][x] has distances (cm)
      //  - heightGrid[y][x] has heights 0..60
      // We can drive 20x20 tactile actuators from heightGrid

      // Press button again to rescan
      if (buttonState == LOW) {
        delay(50);
        if (digitalRead(BUTTON_PIN) == LOW) {
          systemState = SCANNING;
          Serial.println("Button pressed → rescan.");
        }
      }
      break;
  }
}