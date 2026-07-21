/*
  3-DOF Robotic Arm Firmware
  ---------------------------
  Role: pure translator. All IK math happens on the Python side.
  This sketch only:
    1. Reads a comma-separated angle string over serial, e.g. "90,45,120\n"
       (base, shoulder, elbow — angles in degrees, 0-180)
    2. Maps each angle to a PCA9685 pulse width (ticks)
    3. Writes that pulse to the corresponding servo channel

  Hardware:
    - Arduino Uno (ELEGOO clone, CH340 USB)
    - PCA9685 PWM driver @ I2C address 0x40
    - MG995 servos on channels 0 (base), 1 (shoulder), 2 (elbow)
    - Channel 3 reserved for future SG90 claw (not used yet)

  Serial: 115200 baud
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channel assignments
const uint8_t CH_BASE     = 0;
const uint8_t CH_SHOULDER = 1;
const uint8_t CH_ELBOW    = 2;
// const uint8_t CH_CLAW  = 3; // reserved for future SG90
const bool CH_SHOULDER_INVERTED = true;


// PCA9685 pulse range (ticks @ 50 Hz)
// These are the standard MG995 calibration points — adjust after
// physical testing if a servo doesn't hit true 0/180 mechanically.
const int SERVO_MIN = 150; // tick count for 0 degrees
const int SERVO_MAX = 600; // tick count for 180 degrees
const int PWM_FREQ  = 50;  // Hz

// Serial parsing
const uint8_t NUM_JOINTS = 3;
String inputBuffer = "";
bool lineReady = false;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10); // PCA9685 needs a moment after setting frequency

  // Move to a safe neutral pose on startup
  moveJoint(CH_BASE, 90);
  moveJoint(CH_SHOULDER, 90);
  moveJoint(CH_ELBOW, 90);

  Serial.println("READY");
}

void loop() {
  readSerial();

  if (lineReady) {
    processLine(inputBuffer);
    inputBuffer = "";
    lineReady = false;
  }
}

// Non-blocking-ish serial read: accumulate chars until newline
void readSerial() {
  while (Serial.available() > 0 && !lineReady) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        lineReady = true;
      }
    } else {
      inputBuffer += c;
    }
  }
}

// Parses "base,shoulder,elbow" and drives the servos
void processLine(String line) {
  int angles[NUM_JOINTS];
  int startIdx = 0;
  uint8_t found = 0;

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    int commaIdx = line.indexOf(',', startIdx);
    String token;

    if (commaIdx == -1) {
      // last token, goes to end of string
      token = line.substring(startIdx);
    } else {
      token = line.substring(startIdx, commaIdx);
      startIdx = commaIdx + 1;
    }

    token.trim();
    if (token.length() == 0) break;

    angles[i] = token.toInt();
    found++;

    if (commaIdx == -1) break;
  }

  if (found != NUM_JOINTS) {
    Serial.print("ERR bad line: ");
    Serial.println(line);
    return;
  }

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    angles[i] = constrain(angles[i], 0, 180);
  }

  moveJoint(CH_BASE, angles[0]);
  moveJoint(CH_SHOULDER, angles[1]);
  moveJoint(CH_ELBOW, angles[2]);

  // Echo back so Python side can confirm what was applied
  Serial.print("OK ");
  Serial.print(angles[0]); Serial.print(",");
  Serial.print(angles[1]); Serial.print(",");
  Serial.println(angles[2]);
}

// Maps a 0-180 degree angle to PCA9685 ticks and writes it
void moveJoint(uint8_t channel, int angleDeg) {
  if (channel == CH_SHOULDER && CH_SHOULDER_INVERTED) {
    angleDeg = 180 - angleDeg;
  }
  int ticks = map(angleDeg, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, ticks);
}
