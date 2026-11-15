#include <IRremote.h>
#include <Servo.h>
#include <EEPROM.h>

// ---------------- Pins ----------------
#define LM_IN1    2     // IN1
#define LM_IN2    4     // IN2
#define RM_IN3    5     // IN3
#define RM_IN4    7     // IN4

#define RECV_PIN  10    // IR receiver OUT
#define SERVO_PIN 9     // servo signal
#define TRIG_PIN  A1    // HC-SR04 trig
#define ECHO_PIN  A2    // HC-SR04 echo

IRrecv irrecv(RECV_PIN);
decode_results results;
Servo scanner;

// Remote codes
// Codes for YOUR remote
const unsigned long FWD       = 0xFE40BF; // 2
const unsigned long LFT       = 0xFE20DF; // 4
const unsigned long RGT       = 0xFE609F; // 6
const unsigned long BWD       = 0xFE10EF; // 8
const unsigned long STOP      = 0xFE00FF; // 0
const unsigned long RPEAT     = 0xFE50AF; // replay : replay path from RAM
const unsigned long DEL       = 0xFEC837; // exit   : clear RAM path
const unsigned long PERST     = 0xFE08F7; // enter  : save RAM path -> EEPROM
const unsigned long PLAYEPROM = 0xFE42BD; // Home   : load EEPROM path and replay

//Calibration
const unsigned int TURN_MS        = 450;  // time for a 90 degree spin
const unsigned int SIDE_MS        = 700;  // sidestep distance (time)
const unsigned int CLEAR_EXTRA_MS = 500;  // extra forward time after obstacle is passed
const unsigned int MAX_PASS_MS    = 4000; // safety limit while passing the obstacle
const int  OBSTACLE_CM  = 20;   // stop if something is closer than this
const int  SIDE_SEE_CM  = 45;   // side sensor "sees obstacle" below this
const int  SIDE_FREE_CM = 25;   // a side needs at least this much free space
const byte MAX_DETOURS  = 3;    // max detours per forward segment

// Servo angles (swap SERVO_LEFT / SERVO_RIGHT if they look the wrong way)
const int SERVO_FRONT = 90;
const int SERVO_RIGHT = 20;
const int SERVO_LEFT  = 160;

//Path storage
enum { A_FWD = 1, A_LFT = 2, A_RGT = 3, A_BWD = 4 };
#define MAX_STEPS 40

struct Step {
  byte act;
  unsigned int ms;
};

Step steps[MAX_STEPS];
byte stepCount = 0;

byte curAct = 0;              // movement currently being recorded (FWD/BWD)
unsigned long curStart = 0;

//IR debounce
const unsigned int DEBOUNCE_MS = 300;
unsigned long lastCode = 0;
unsigned long lastCodeTime = 0;
bool hasLastCode = false;


void setup() {
  pinMode(LM_IN1, OUTPUT);
  pinMode(LM_IN2, OUTPUT);
  pinMode(RM_IN3, OUTPUT);
  pinMode(RM_IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  motorsStop();

  Serial.begin(9600);
  scanner.attach(SERVO_PIN);
  scanner.write(SERVO_FRONT);

  irrecv.enableIRIn();
  Serial.println(F("Robot ready"));
}

void loop() {
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;
    irrecv.resume();
    if (code != 0xFFFFFFFF) {                   // ignore "repeat" codes
      unsigned long now = millis();
      bool duplicate = hasLastCode && code == lastCode &&
                       (now - lastCodeTime) < DEBOUNCE_MS;
      lastCode = code;
      lastCodeTime = now;                       // a held button keeps extending this
      hasLastCode = true;

      if (!duplicate) {
        Serial.print(F("IR code 0x"));
        Serial.println(code, HEX);
        handleCode(code);
        lastCodeTime = millis();                // ignore repeats that arrived during a long action
      }
    }
  }
}

void handleCode(unsigned long code) {
  switch (code) {
    case FWD:
      closeCurrent();
      curAct = A_FWD; curStart = millis();
      motorsForward();
      break;
    case BWD:
      closeCurrent();
      curAct = A_BWD; curStart = millis();
      motorsBackward();
      break;
    case LFT:
      closeCurrent();
      turn(false);
      addStep(A_LFT, TURN_MS);
      break;
    case RGT:
      closeCurrent();
      turn(true);
      addStep(A_RGT, TURN_MS);
      break;
    case STOP:
      closeCurrent();
      break;
    case RPEAT:
      closeCurrent();
      playPath();
      break;
    case DEL:
      closeCurrent();
      stepCount = 0;
      Serial.println(F("Path cleared"));
      break;
    case PERST:
      closeCurrent();
      saveToEEPROM();
      break;
    case PLAYEPROM:
      closeCurrent();
      loadFromEEPROM();
      playPath();
      break;
  }
}

// =====================================================================
//                         RECORDING / STORAGE
// =====================================================================
void addStep(byte act, unsigned long ms) {
  if (ms > 65535UL) ms = 65535UL;
  if (stepCount >= MAX_STEPS) {
    Serial.println(F("Path memory full"));
    return;
  }
  steps[stepCount].act = act;
  steps[stepCount].ms  = (unsigned int)ms;
  stepCount++;
  Serial.print(F("Step ")); Serial.print(stepCount);
  Serial.print(F(": act=")); Serial.print(act);
  Serial.print(F(" ms="));   Serial.println(ms);
}

// Ends the FWD/BWD movement that is being recorded
void closeCurrent() {
  if (curAct == A_FWD || curAct == A_BWD) {
    unsigned long d = millis() - curStart;
    if (d >= 100) addStep(curAct, d);     // ignore accidental taps
  }
  curAct = 0;
  motorsStop();
}

// EEPROM layout: [0] = step count, then 3 bytes per step (act, ms low, ms high)
void saveToEEPROM() {
  EEPROM.update(0, stepCount);
  for (byte i = 0; i < stepCount; i++) {
    int a = 1 + i * 3;
    EEPROM.update(a, steps[i].act);
    EEPROM.update(a + 1, steps[i].ms & 0xFF);
    EEPROM.update(a + 2, steps[i].ms >> 8);
  }
  Serial.println(F("Path saved to EEPROM"));
}

void loadFromEEPROM() {
  byte n = EEPROM.read(0);
  if (n > MAX_STEPS) n = 0;               // empty / garbage EEPROM
  stepCount = n;
  for (byte i = 0; i < n; i++) {
    int a = 1 + i * 3;
    steps[i].act = EEPROM.read(a);
    steps[i].ms  = EEPROM.read(a + 1) | (EEPROM.read(a + 2) << 8);
  }
  Serial.print(F("Loaded steps: ")); Serial.println(n);
}

// =====================================================================
//                              REPLAY
// =====================================================================
void playPath() {
  Serial.println(F("Replay start"));
  scanner.write(SERVO_FRONT);
  delay(300);

  for (byte i = 0; i < stepCount; i++) {
    byte act = steps[i].act;
    unsigned long ms = steps[i].ms;
    bool ok = true;

    if      (act == A_FWD) ok = forwardWithAvoidance(ms);
    else if (act == A_BWD) ok = backwardTimed(ms);
    else if (act == A_LFT) turn(false);
    else if (act == A_RGT) turn(true);

    if (!ok) {
      motorsStop();
      Serial.println(F("Replay aborted"));
      return;
    }
    delay(150);
  }
  motorsStop();
  Serial.println(F("Destination reached"));
}

// Drives forward `remaining` ms, detouring around obstacles. false = abort.
bool forwardWithAvoidance(unsigned long remaining) {
  byte detours = 0;

  while (remaining > 0) {
    bool blocked, aborted;
    unsigned long d = driveChecked(remaining, blocked, aborted);
    if (aborted) return false;
    if (!blocked) return true;                       // finished this segment

    remaining = (d >= remaining) ? 0 : remaining - d;
    if (remaining == 0) return true;

    Serial.println(F("Obstacle! detouring"));
    if (++detours > MAX_DETOURS) return false;

    unsigned long passed = 0;
    if (!detour(passed)) return false;
    remaining = (passed >= remaining) ? 0 : remaining - passed;   // progress made while passing
  }
  return true;
}

// Forward until time is up, obstacle is seen, or STOP is pressed
unsigned long driveChecked(unsigned long ms, bool &blocked, bool &aborted) {
  unsigned long start = millis();
  blocked = false; aborted = false;
  motorsForward();
  while (millis() - start < ms) {
    if (stopPressed()) { aborted = true; break; }
    if (obstacleAhead()) { blocked = true; break; }
    delay(20);
  }
  motorsStop();
  return millis() - start;
}

bool backwardTimed(unsigned long ms) {      // no rear sensor: plain timed move
  unsigned long start = millis();
  motorsBackward();
  while (millis() - start < ms) {
    if (stopPressed()) { motorsStop(); return false; }
    delay(20);
  }
  motorsStop();
  return true;
}

/*
  Detour (obstacle on the path, car is stopped in front of it):
   1 scan left/right, pick the freer side
   2 turn to that side, sidestep
   3 turn back to original heading
   4 drive forward while the servo looks at the obstacle side,
     until the obstacle was seen and then is gone (+ small extra)
   5 turn toward the original line, sidestep back, turn to original heading
  `passed` = forward time (along the original path) travelled in step 4.
*/
bool detour(unsigned long &passed) {
  passed = 0;
  motorsStop();

  // 1. scan
  int distR = lookAt(SERVO_RIGHT);
  int distL = lookAt(SERVO_LEFT);
  scanner.write(SERVO_FRONT);
  delay(350);
  Serial.print(F("Right=")); Serial.print(distR);
  Serial.print(F(" Left="));  Serial.println(distL);

  if (distR < SIDE_FREE_CM && distL < SIDE_FREE_CM) {
    Serial.println(F("Boxed in"));
    return false;
  }
  bool goRight = (distR >= distL);

  // 2. sidestep away from the line
  turn(goRight);
  if (!moveFixed(SIDE_MS)) return false;

  // 3. face the original heading again
  turn(!goRight);

  // 4. pass the obstacle - servo looks toward the obstacle side
  scanner.write(goRight ? SERVO_LEFT : SERVO_RIGHT);
  delay(400);

  unsigned long t0 = millis();
  bool seen = false;
  byte clearCount = 0;
  motorsForward();
  while (millis() - t0 < MAX_PASS_MS) {
    if (stopPressed()) { motorsStop(); scanner.write(SERVO_FRONT); return false; }
    int d = readDistanceCm();
    if (d < SIDE_SEE_CM) { seen = true; clearCount = 0; }
    else if (seen) {
      if (++clearCount >= 3) break;          // obstacle is now behind the side sensor
    }
    delay(30);
  }
  if (seen) delay(CLEAR_EXTRA_MS);           // so the car's tail clears it too
  motorsStop();
  passed = millis() - t0;                    // includes the CLEAR_EXTRA_MS time

  scanner.write(SERVO_FRONT);
  delay(400);

  // 5. return to the original line
  turn(!goRight);
  if (!moveFixed(SIDE_MS)) return false;
  turn(goRight);

  Serial.println(F("Back on path"));
  return true;
}

// Forward for a fixed time with front obstacle check. false = blocked/aborted
bool moveFixed(unsigned long ms) {
  bool blocked, aborted;
  driveChecked(ms, blocked, aborted);
  return !(blocked || aborted);
}

int lookAt(int angle) {
  scanner.write(angle);
  delay(550);
  int a = readDistanceCm();
  delay(40);
  int b = readDistanceCm();
  return (a + b) / 2;
}

// =====================================================================
//                         SENSORS / INPUT HELPERS
// =====================================================================
int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long t = pulseIn(ECHO_PIN, HIGH, 25000UL);  // ~4 m timeout
  if (t == 0) return 999;                      // no echo = nothing close
  return (int)(t / 58);
}

bool obstacleAhead() {
  if (readDistanceCm() >= OBSTACLE_CM) return false;
  delay(10);
  return readDistanceCm() < OBSTACLE_CM;       // confirm with a 2nd reading
}

bool stopPressed() {
  if (irrecv.decode(&results)) {
    unsigned long c = results.value;
    irrecv.resume();
    return (c == STOP);
  }
  return false;
}

// =====================================================================
//                              MOTORS (L298N)
// =====================================================================
// l / r : 1 = forward, -1 = backward, 0 = stop
void setMotors(int l, int r) {
  digitalWrite(LM_IN1, l == 1);
  digitalWrite(LM_IN2, l == -1);
  digitalWrite(RM_IN3, r == 1);
  digitalWrite(RM_IN4, r == -1);
}

void motorsForward()  { setMotors(1, 1); }
void motorsBackward() { setMotors(-1, -1); }
void motorsStop()     { setMotors(0, 0); }

// Spin in place ~90 degrees
void turn(bool right) {
  motorsStop();
  delay(80);
  if (right) setMotors(1, -1);
  else       setMotors(-1, 1);
  delay(TURN_MS);
  motorsStop();
  delay(150);
}
