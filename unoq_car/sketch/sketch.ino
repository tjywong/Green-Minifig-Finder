// Green Minifig Finder car - MCU side of the Arduino UNO Q App Lab app.
//
// Drives two DC motors through a Cytron MakerDrive. The Linux side (python/main.py)
// decides the speed and calls set_motors(left, right) over the Bridge about 20x/second.
//
// MakerDrive truth table (per motor):
//   A = PWM, B = LOW  -> forward
//   A = LOW, B = PWM  -> backward
//   A = LOW, B = LOW  -> brake
//
// Wiring: UNO Q D5 -> M1A, D6 -> M1B, D9 -> M2A, D10 -> M2B, and UNO Q GND -> MakerDrive GND.

#include <Arduino_RouterBridge.h>

const int MOTOR1_A = 5;
const int MOTOR1_B = 6;
const int MOTOR2_A = 9;
const int MOTOR2_B = 10;

// The two motors face opposite ways on most cars, so one usually has to be reversed for
// both wheels to roll forward. Flip these if a wheel spins the wrong way.
const bool MOTOR1_REVERSED = false;
const bool MOTOR2_REVERSED = false;

// Stop the motors if no command arrives for this long (e.g. Wi-Fi drops or Python crashes).
const unsigned long WATCHDOG_MS = 500;

unsigned long lastCommandMs = 0;
bool motorsRunning = false;

// speed: -255 (full backward) .. 0 (brake) .. 255 (full forward)
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
void set_motors(int motor1Speed, int motor2Speed) {
  driveMotor(MOTOR1_A, MOTOR1_B, MOTOR1_REVERSED, motor1Speed);
  driveMotor(MOTOR2_A, MOTOR2_B, MOTOR2_REVERSED, motor2Speed);
  motorsRunning = (motor1Speed != 0 || motor2Speed != 0);
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
