// 00_tests.ino
// Unit tests for scanner.ino without modifying scanner.ino
// Runs on-board and prints results to Serial Monitor.

#define ARDUINO_TESTS 1

// ---------- Hardware stubs (used by scanner.ino via macros) ----------
static int g_fakeButtonRead = HIGH;   // HIGH = not pressed, LOW = pressed

int fakeDigitalRead(uint8_t /*pin*/) {
  return g_fakeButtonRead;
}

xstatic unsigned long g_fakePulseIn = 0;
unsigned long fakePulseIn(uint8_t /*pin*/, uint8_t /*state*/, unsigned long /*timeout*/) {
  return g_fakePulseIn;
}
void fakeDigitalWrite(uint8_t /*pin*/, uint8_t /*val*/) {}
void fakeDelayMicroseconds(unsigned int /*us*/) {}

// ---------- Macro-shim: rename scanner.ino's setup/loop + redirect IO ----------
#define setup scanner_setup
#define loop  scanner_loop

#define digitalRead       fakeDigitalRead
#define pulseIn           fakePulseIn
#define digitalWrite      fakeDigitalWrite
#define delayMicroseconds fakeDelayMicroseconds

// IMPORTANT:
// Do NOT #include "scanner.ino".
// Arduino IDE concatenates .ino tabs; these macros must appear BEFORE scanner.ino code.

// ---------- Tiny test framework ----------
static int g_pass = 0;
static int g_fail = 0;

void assertTrue(const __FlashStringHelper* name, bool cond) {
  if (cond) {
    g_pass++;
    Serial.print(F("[PASS] "));
    Serial.println(name);
  } else {
    g_fail++;
    Serial.print(F("[FAIL] "));
    Serial.println(name);
  }
}

void assertEqLong(const __FlashStringHelper* name, long a, long b) {
  if (a == b) {
    g_pass++;
    Serial.print(F("[PASS] "));
    Serial.println(name);
  } else {
    g_fail++;
    Serial.print(F("[FAIL] "));
    Serial.print(name);
    Serial.print(F(" expected="));
    Serial.print(b);
    Serial.print(F(" got="));
    Serial.println(a);
  }
}

void assertEqInt(const __FlashStringHelper* name, int a, int b) {
  if (a == b) {
    g_pass++;
    Serial.print(F("[PASS] "));
    Serial.println(name);
  } else {
    g_fail++;
    Serial.print(F("[FAIL] "));
    Serial.print(name);
    Serial.print(F(" expected="));
    Serial.print(b);
    Serial.print(F(" got="));
    Serial.println(a);
  }
}

// ---------- Tests ----------

// After scanner.ino is concatenated, these globals/functions exist:
// GRID_SIZE, ANGLE_X_MIN/MAX, ANGLE_Y_MIN/MAX, MAX_HEIGHT_DEG,
// DIST_MAX_CM, STEPS_PER_DEG_X/Y, lastToggleTime, gMinDist, gMaxDist,
// outOfBoundsAngle(), gridToAngleX/Y(), calcAngleSteps(), height(), add(), systemToggle().

void test_outOfBoundsAngle() {
  assertTrue(F("outOfBoundsAngle(0)==false"),  outOfBoundsAngle(0) == false);
  assertTrue(F("outOfBoundsAngle(180)==false"), outOfBoundsAngle(180) == false);
  assertTrue(F("outOfBoundsAngle(181)==true"),  outOfBoundsAngle(181) == true);
}

void test_gridToAngle() {
  assertEqInt(F("gridToAngleX(0)==ANGLE_X_MIN"), gridToAngleX(0), ANGLE_X_MIN);
  assertEqInt(F("gridToAngleX(last)==ANGLE_X_MAX"), gridToAngleX(GRID_SIZE - 1), ANGLE_X_MAX);

  assertEqInt(F("gridToAngleY(0)==ANGLE_Y_MIN"), gridToAngleY(0), ANGLE_Y_MIN);
  assertEqInt(F("gridToAngleY(last)==ANGLE_Y_MAX"), gridToAngleY(GRID_SIZE - 1), ANGLE_Y_MAX);
}

void test_calcAngleSteps() {
  // currentSteps=0 at x=0 should target 0-ish
  long d0 = calcAngleSteps(0, 0, 0, true);
  assertEqLong(F("calcAngleSteps X at (0) from 0 == 0"), d0, 0);

  // At max x, expected delta ~ round(angle * stepsPerDeg)
  int angle = gridToAngleX(GRID_SIZE - 1); // should be ANGLE_X_MAX
  long expectedTarget = (long)(angle * STEPS_PER_DEG_X + 0.5f);
  long d1 = calcAngleSteps(0, GRID_SIZE - 1, 0, true);
  assertEqLong(F("calcAngleSteps X at (last) from 0 == targetSteps"), d1, expectedTarget);
}

void test_heightMapping() {
  // Set global min/max for deterministic test
  gMinDist = 10.0f;
  gMaxDist = 110.0f;

  // Near = max height
  int hNear = height(10);
  assertEqInt(F("height(minDist)==MAX_HEIGHT_DEG"), hNear, MAX_HEIGHT_DEG);

  // Far = 0 height
  int hFar = height(110);
  assertEqInt(F("height(maxDist)==0"), hFar, 0);

  // Background (very far) = 0
  int hBg = height((unsigned long)DIST_MAX_CM);
  assertEqInt(F("height(DIST_MAX_CM)==0"), hBg, 0);
}

void test_addBounds() {
  // Ensure known state
  add(0, 0, 7);
  int before = heightGrid[0][0];

  // Out of bounds write should do nothing
  add(GRID_SIZE, 0, 55);
  int after = heightGrid[0][0];

  assertEqInt(F("add(out-of-bounds) does not overwrite"), after, before);
}

void test_systemToggleDebounceAndEdge() {
  // Reset global debounce timer used in scanner.ino
  lastToggleTime = 0;

  bool status = false;

  // Not pressed
  g_fakeButtonRead = HIGH;
  status = systemToggle(status, 1000, 200);
  assertTrue(F("toggle: no press => stays OFF"), status == false);

  // Press (edge)
  g_fakeButtonRead = LOW;
  status = systemToggle(status, 1100, 200);
  assertTrue(F("toggle: press edge => turns ON"), status == true);

  // Still held: should NOT toggle again
  status = systemToggle(status, 1150, 200);
  assertTrue(F("toggle: held => stays ON"), status == true);

  // Release
  g_fakeButtonRead = HIGH;
  status = systemToggle(status, 1200, 200);
  assertTrue(F("toggle: release => stays ON"), status == true);

  // Press again too soon (<200ms since last toggle at 1100): should NOT toggle
  g_fakeButtonRead = LOW;
  status = systemToggle(status, 1250, 200);
  assertTrue(F("toggle: press within debounce => stays ON"), status == true);

  // Release, wait enough, press again => toggles OFF
  g_fakeButtonRead = HIGH;
  status = systemToggle(status, 1400, 200);
  g_fakeButtonRead = LOW;
  status = systemToggle(status, 1500, 200);
  assertTrue(F("toggle: press after debounce => turns OFF"), status == false);
}

void runAllTests() {
  g_pass = 0;
  g_fail = 0;

  Serial.println(F("===== UNIT TESTS START ====="));

  test_outOfBoundsAngle();
  test_gridToAngle();
  test_calcAngleSteps();
  test_heightMapping();
  test_addBounds();
  test_systemToggleDebounceAndEdge();

  Serial.println(F("===== UNIT TESTS END ====="));
  Serial.print(F("PASS: "));
  Serial.println(g_pass);
  Serial.print(F("FAIL: "));
  Serial.println(g_fail);

  if (g_fail == 0) Serial.println(F("✅ ALL TESTS PASSED"));
  else             Serial.println(F("❌ SOME TESTS FAILED"));
}

#undef setup
#undef loop

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* for boards that need it */ }
  delay(500);

  runAllTests();
}

void loop() {
  // Nothing; tests run once.
}
