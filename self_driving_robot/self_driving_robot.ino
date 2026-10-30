/*
 * Self-Driving Maze Robot
 *
 * An Arduino robot that drives itself through a maze using three ultrasonic
 * sensors (front, left, right) and two continuous-rotation servos for the
 * wheels. It follows walls, picks the more open path at junctions, and turns
 * around at dead ends.
 *
 * Hardware
 *   - Arduino Uno (or compatible)
 *   - 3x ultrasonic sensors in single-pin mode (trigger and echo on one pin)
 *       front -> A2, left -> A0, right -> A1
 *   - 2x continuous-rotation servos: right wheel -> D12, left wheel -> D13
 *   - 1x standard servo for the arm -> D11 (parked at startup)
 *
 * Libraries
 *   - Servo   (bundled with the Arduino IDE)
 *   - NewPing (Library Manager: "NewPing" by Tim Eckel)
 */

#include <Servo.h>
#include <NewPing.h>

// ---------------------------------------------------------------- pins

const uint8_t PIN_SONAR_FRONT = A2;
const uint8_t PIN_SONAR_LEFT  = A0;
const uint8_t PIN_SONAR_RIGHT = A1;

const uint8_t PIN_SERVO_ARM   = 11;
const uint8_t PIN_WHEEL_RIGHT = 12;
const uint8_t PIN_WHEEL_LEFT  = 13;

// ---------------------------------------------------------------- tuning

const unsigned int MAX_DISTANCE_CM = 250;  // NewPing gives up beyond this
const unsigned int FRONT_STOP_CM   = 30;   // wall ahead closer than this: stop and decide
const unsigned int SIDE_OPEN_CM    = 30;   // a side further than this counts as an opening
const unsigned int SIDE_TARGET_CM  = 12;   // preferred gap to the wall being followed
const unsigned int SIDE_DEADBAND   = 3;    // ignore drift smaller than this

// Servo pulse widths (microseconds). 1500 is stop for a calibrated
// continuous-rotation servo. These values come from the original robot.
const int PULSE_STOP     = 1500;
const int PULSE_FAST_FWD = 200;   // offset from stop for full speed
const int PULSE_TRIM     = 60;    // offset reduction used to steer while driving

// Turn timings (ms) measured on the original robot.
const unsigned long TURN_90_MS  = 1400;
const unsigned long TURN_180_MS = 2800;
const unsigned long SETTLE_MS   = 100;

const uint8_t PING_SAMPLES = 3;   // median of N pings to reject stray echoes

// Set to true to run each motion once at startup so you can confirm the
// wheels turn the way the function names say before running the maze.
const bool RUN_DIRECTION_TEST = false;

// ---------------------------------------------------------------- hardware

NewPing sonarFront(PIN_SONAR_FRONT, PIN_SONAR_FRONT, MAX_DISTANCE_CM);
NewPing sonarLeft (PIN_SONAR_LEFT,  PIN_SONAR_LEFT,  MAX_DISTANCE_CM);
NewPing sonarRight(PIN_SONAR_RIGHT, PIN_SONAR_RIGHT, MAX_DISTANCE_CM);

Servo wheelRight;
Servo wheelLeft;
Servo arm;

// ---------------------------------------------------------------- motion
//
// The two wheel servos face opposite directions, so "forward" is a pulse
// above 1500 on the right wheel and below 1500 on the left wheel.

void drive(int rightOffset, int leftOffset) {
  wheelRight.writeMicroseconds(PULSE_STOP + rightOffset);
  wheelLeft.writeMicroseconds(PULSE_STOP - leftOffset);
}

void stopWheels()  { drive(0, 0); }
void goForwards()  { drive(PULSE_FAST_FWD, PULSE_FAST_FWD); }
void goBackwards() { drive(-PULSE_FAST_FWD, -PULSE_FAST_FWD); }

// Pivot turns: one wheel drives, the other holds. These match the original
// turnLeft()/turnRight() pulse values that ran the maze.
void turnLeft()  { drive(0, PULSE_FAST_FWD); }
void turnRight() { drive(PULSE_FAST_FWD, 0); }

// Spin in place: wheels run in opposite directions (used for U-turns).
void spinAround() { drive(-PULSE_FAST_FWD, PULSE_FAST_FWD); }

// Drive forward while nudging toward or away from a wall.
// steer > 0 bends toward the turnRight() side, steer < 0 toward turnLeft().
void driveSteered(int steer) {
  int right = PULSE_FAST_FWD;
  int left  = PULSE_FAST_FWD;
  if (steer > 0) left  -= PULSE_TRIM;
  if (steer < 0) right -= PULSE_TRIM;
  drive(right, left);
}

void timedMove(void (*move)(), unsigned long ms) {
  move();
  delay(ms);
  stopWheels();
  delay(SETTLE_MS);
}

// ---------------------------------------------------------------- sensing

// NewPing returns 0 when nothing echoes back within MAX_DISTANCE_CM.
// That means "open", not "wall at 0 cm", so map it to the max range.
unsigned int readCm(NewPing &sonar) {
  unsigned long us = sonar.ping_median(PING_SAMPLES);
  unsigned int cm = NewPing::convert_cm(us);
  return cm == 0 ? MAX_DISTANCE_CM : cm;
}

struct Readings {
  unsigned int front;
  unsigned int left;
  unsigned int right;
};

Readings readAll() {
  Readings r;
  r.front = readCm(sonarFront);
  delay(30);                      // let echoes die out between sensors
  r.left  = readCm(sonarLeft);
  delay(30);
  r.right = readCm(sonarRight);
  return r;
}

void logReadings(const Readings &r, const char *action) {
  Serial.print(F("front="));  Serial.print(r.front);
  Serial.print(F(" left="));  Serial.print(r.left);
  Serial.print(F(" right=")); Serial.print(r.right);
  Serial.print(F("  -> "));   Serial.println(action);
}

// ---------------------------------------------------------------- decisions

enum Action { FORWARD, TURN_LEFT, TURN_RIGHT, TURN_AROUND };

Action decide(const Readings &r) {
  if (r.front >= FRONT_STOP_CM) return FORWARD;

  bool leftOpen  = r.left  >= SIDE_OPEN_CM;
  bool rightOpen = r.right >= SIDE_OPEN_CM;

  // Original rule kept: if the left side is blocked, take the other side.
  if (!leftOpen && !rightOpen) return TURN_AROUND;
  if (!leftOpen)  return TURN_RIGHT;
  if (!rightOpen) return TURN_LEFT;

  // Both sides open: head toward the roomier one.
  return (r.left >= r.right) ? TURN_LEFT : TURN_RIGHT;
}

// Keep a steady gap to the nearer wall while driving straight.
int wallCorrection(const Readings &r) {
  bool leftWall  = r.left  < SIDE_OPEN_CM;
  bool rightWall = r.right < SIDE_OPEN_CM;

  if (leftWall && rightWall) {
    int diff = (int)r.left - (int)r.right;   // positive: closer to the right wall
    if (abs(diff) <= (int)SIDE_DEADBAND) return 0;
    return diff > 0 ? -1 : 1;
  }
  if (leftWall) {
    int err = (int)r.left - (int)SIDE_TARGET_CM;
    if (abs(err) <= (int)SIDE_DEADBAND) return 0;
    return err > 0 ? -1 : 1;                 // too far: bend toward it
  }
  if (rightWall) {
    int err = (int)r.right - (int)SIDE_TARGET_CM;
    if (abs(err) <= (int)SIDE_DEADBAND) return 0;
    return err > 0 ? 1 : -1;
  }
  return 0;
}

// ---------------------------------------------------------------- setup / loop

void directionTest() {
  Serial.println(F("Direction test: forward, back, left, right, spin"));
  timedMove(goForwards, 800);
  timedMove(goBackwards, 800);
  timedMove(turnLeft, TURN_90_MS);
  timedMove(turnRight, TURN_90_MS);
  timedMove(spinAround, TURN_180_MS);
  Serial.println(F("Direction test done"));
}

void setup() {
  Serial.begin(9600);

  wheelRight.attach(PIN_WHEEL_RIGHT);
  wheelLeft.attach(PIN_WHEEL_LEFT);
  arm.attach(PIN_SERVO_ARM);

  arm.write(180);                 // park the arm out of the way
  stopWheels();
  delay(1000);                    // time to set the robot down

  if (RUN_DIRECTION_TEST) directionTest();
}

void loop() {
  Readings r = readAll();

  switch (decide(r)) {
    case FORWARD: {
      int steer = wallCorrection(r);
      logReadings(r, steer == 0 ? "forward" : (steer > 0 ? "forward, nudge right" : "forward, nudge left"));
      if (steer == 0) goForwards(); else driveSteered(steer);
      delay(SETTLE_MS);
      break;
    }
    case TURN_LEFT:
      logReadings(r, "turn left");
      stopWheels(); delay(SETTLE_MS);
      timedMove(turnLeft, TURN_90_MS);
      break;
    case TURN_RIGHT:
      logReadings(r, "turn right");
      stopWheels(); delay(SETTLE_MS);
      timedMove(turnRight, TURN_90_MS);
      break;
    case TURN_AROUND:
      logReadings(r, "dead end, turn around");
      stopWheels(); delay(SETTLE_MS);
      timedMove(goBackwards, 300);
      timedMove(spinAround, TURN_180_MS);
      break;
  }
}
