// Green Minifig Finder car - MCU side of the Arduino UNO Q App Lab app.
//
// Drives two DC motors through a Cytron MakerDrive. The Linux side (python/main.py)
// calls set_motors(left, right) over the Bridge about 20x/second; this sketch only
// turns those signed speeds into PWM and stops the car if the commands stop arriving.
//
// MakerDrive truth table (per motor, inputs A and B):
//   A = PWM, B = LOW  -> forward
//   A = LOW, B = PWM  -> backward
//   A = LOW, B = LOW  -> stop
//
// Wiring: D5 -> M1A, D6 -> M1B (left motor), D9 -> M2A, D10 -> M2B (right motor),
// and UNO Q GND -> MakerDrive GND.

#include <Arduino_RouterBridge.h>

const int MOTOR1_A = 5;   // left motor
const int MOTOR1_B = 6;
const int MOTOR2_A = 9;   // right motor
const int MOTOR2_B = 10;

// Flip these if a wheel spins the wrong way when the car is told to drive forward.
const bool MOTOR1_REVERSED = false;
const bool MOTOR2_REVERSED = true;

// Stop if no command arrives for this long (Wi-Fi drop, Python crash, ...).
const unsigned long WATCHDOG_MS = 400;

unsigned long lastCommandMs = 0;
bool motorsRunning = false;

// speed: -255 (full backward) .. 0 (stop) .. 255 (full forward)
void driveMotor(int pinA, int pinB, bool reversed, int speed) {
  speed = constrain(speed, -255, 255);
  if (reversed) {
    speed = -speed;
  }

  if (speed > 0) {
    analogWrite(pinB, 0);
    analogWrite(pinA, speed);
  } else if (speed < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -speed);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void stopMotors() {
  driveMotor(MOTOR1_A, MOTOR1_B, MOTOR1_REVERSED, 0);
  driveMotor(MOTOR2_A, MOTOR2_B, MOTOR2_REVERSED, 0);
  motorsRunning = false;
}

// Called from Python: Bridge.call("set_motors", left, right)
void set_motors(int left, int right) {
  driveMotor(MOTOR1_A, MOTOR1_B, MOTOR1_REVERSED, left);
  driveMotor(MOTOR2_A, MOTOR2_B, MOTOR2_REVERSED, right);
  motorsRunning = (left != 0 || right != 0);
  lastCommandMs = millis();
}

void setup() {
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  stopMotors();

  Bridge.begin();
  Monitor.begin();
  // provide_safe runs set_motors in the main loop thread, so it never races loop()
  Bridge.provide_safe("set_motors", set_motors);
}

void loop() {
  if (motorsRunning && millis() - lastCommandMs > WATCHDOG_MS) {
    stopMotors();
    Monitor.println("Watchdog: no motor command received, motors stopped");
  }
}
