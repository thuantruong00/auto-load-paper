#include <Arduino.h>

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

// Pin mapping
const int MOTOR_PWM_PIN = 25; // Tín hiệu PWM đến module MOSFET, kích mức thấp
const int PAPER_SENSOR_PIN = 34; // DO module quang trở
const int SERVO_PIN = 13;
const int SERVO_CHANNEL = 0;
const int MOTOR_PWM_CHANNEL = 1;
const int SERVO_FREQUENCY = 50;
const int SERVO_RESOLUTION = 16;
const int MOTOR_PWM_FREQUENCY = 1000;
const int MOTOR_PWM_RESOLUTION = 8;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const int SERVO_MIN_ANGLE = 90;
const int SERVO_MAX_ANGLE = 130;
const int SERVO_START_ANGLE = 90;
const int SERVO_ANGLE_STEP = 1;
const unsigned long SERVO_MOVE_INTERVAL_MS = 20;
const int PAPER_DETECTED_LEVEL = LOW;

String serialCommand;
int motorSpeedPercent = 0;
int servoCurrentAngle = SERVO_START_ANGLE;
int servoTargetAngle = SERVO_START_ANGLE;
unsigned long lastServoMoveMs = 0;

void setup() {
  Serial.begin(115200);

  setupMotorPwm();
  setMotorSpeed(0);
  pinMode(PAPER_SENSOR_PIN, INPUT);
  setupServoPwm();
  servoCurrentAngle = SERVO_START_ANGLE;
  servoTargetAngle = SERVO_START_ANGLE;
  writeServoAngle(servoCurrentAngle);

  Serial.println("Ready. Nhap help: de xem lenh.");
}

void loop() {
  readSerialCommand();
  updateServoMotion();
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialCommand.length() > 0) {
        handleCommand(serialCommand);
        serialCommand = "";
      }
    } else {
      serialCommand += c;
    }
  }
}

void handleCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command == "help") {
    Serial.printf("servo:<%d-%d>\n", SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    Serial.println("motor:<0-100>");
    Serial.println("motor:0 = stop");
    Serial.println("sensor");
    Serial.println("stop");
    Serial.println("status");
    Serial.printf("Motor PWM hien tai: %d%%\n", motorSpeedPercent);
    return;
  }

  if (command == "stop") {
    setMotorSpeed(0);
    Serial.println("Motor stopped");
    return;
  }

  if (command == "status") {
    printStatus();
    return;
  }

  if (command == "sensor") {
    printSensorStatus();
    return;
  }

  if (command.startsWith("motor:")) {
    int requestedPercent = command.substring(6).toInt();
    setMotorSpeed(requestedPercent);
    Serial.printf("Motor PWM: %d%%\n", motorSpeedPercent);
    return;
  }

  if (command.startsWith("servo:")) {
    int angle = command.substring(6).toInt();
    servoTargetAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    Serial.printf("Servo target: %d degrees, current: %d degrees\n",
                  servoTargetAngle, servoCurrentAngle);
    return;
  }

  Serial.println("Lenh khong hop le. Goi help de xem danh sach lenh.");
}

void printStatus() {
  Serial.printf("motor_pwm=%d%%, output=%d\n",
                motorSpeedPercent, motorSpeedPercent > 0 ? 1 : 0);
  Serial.printf("servo_current=%d, servo_target=%d\n",
                servoCurrentAngle, servoTargetAngle);
  printSensorStatus();
}

void printSensorStatus() {
  int sensorLevel = digitalRead(PAPER_SENSOR_PIN);
  bool paperDetected = sensorLevel == PAPER_DETECTED_LEVEL;
  Serial.printf("sensor_do=%d, paper=%s\n",
                sensorLevel, paperDetected ? "YES" : "NO");
}

void updateServoMotion() {
  unsigned long now = millis();

  if (now - lastServoMoveMs < SERVO_MOVE_INTERVAL_MS) {
    return;
  }

  lastServoMoveMs = now;

  if (servoCurrentAngle == servoTargetAngle) {
    return;
  }

  if (servoCurrentAngle < servoTargetAngle) {
    servoCurrentAngle = min(servoCurrentAngle + SERVO_ANGLE_STEP,
                            servoTargetAngle);
  } else {
    servoCurrentAngle = max(servoCurrentAngle - SERVO_ANGLE_STEP,
                            servoTargetAngle);
  }

  writeServoAngle(servoCurrentAngle);
}

void setupMotorPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_PWM_PIN, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
#else
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL);
#endif
}

void setMotorSpeed(int percent) {
  motorSpeedPercent = constrain(percent, 0, 100);
  int requestedDuty = map(motorSpeedPercent, 0, 100, 0, 255);
  int outputDuty = 255 - requestedDuty; // Module kích mức thấp qua GND

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM_PIN, outputDuty);
#else
  ledcWrite(MOTOR_PWM_CHANNEL, outputDuty);
#endif
}

void setupServoPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, SERVO_FREQUENCY, SERVO_RESOLUTION);
#else
  ledcSetup(SERVO_CHANNEL, SERVO_FREQUENCY, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
#endif
}

void writeServoAngle(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  unsigned long pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  unsigned long maxDuty = (1UL << SERVO_RESOLUTION) - 1;
  unsigned long duty = (pulseUs * maxDuty) / 20000UL;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(SERVO_CHANNEL, duty);
#endif
}
