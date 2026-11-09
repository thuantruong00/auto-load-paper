#include <Arduino.h>

// 0: tat log debug; 1: bat log debug
#define DEBUG_SERIAL 0

#if DEBUG_SERIAL
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

// Pin mapping
const int MOTOR_PWM_PIN = 25; // Tín hiệu PWM đến module MOSFET, kích mức thấp
const int PAPER_SENSOR_PIN = 34; // DO module quang trở
const int SERVO_PIN = 13;
const int PUSH_RPWM_PIN = 26; // BTS7960: quay tien
const int PUSH_LPWM_PIN = 27; // BTS7960: quay lui
const int LIMIT_FORWARD_PIN = 19; // PC817 opto1: gioi han huong +
const int LIMIT_REVERSE_PIN = 18;  // PC817 opto2: gioi han huong -
const int RELAY_PIN = 5; // Module relay: muc 0/1 theo lenh Serial
const unsigned long RELAY_RESET_TIME_MS = 500UL;
const int SERVO_CHANNEL = 0;
const int MOTOR_PWM_CHANNEL = 1;
const int PUSH_RPWM_CHANNEL = 2;
const int PUSH_LPWM_CHANNEL = 3;
const int SERVO_FREQUENCY = 50;
const int SERVO_RESOLUTION = 16;
const int MOTOR_PWM_FREQUENCY = 1000;
const int MOTOR_PWM_RESOLUTION = 8;
const int PUSH_PWM_FREQUENCY = 1000;
const int PUSH_PWM_RESOLUTION = 8;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const int SERVO_MIN_ANGLE = 90;
const int SERVO_MAX_ANGLE = 150;
const int SERVO_START_ANGLE = 90;
const int SERVO_ANGLE_STEP = 1;
const unsigned long SERVO_MOVE_INTERVAL_MS = 20;
const int PAPER_DETECTED_LEVEL = LOW;
const unsigned long PAPER_SENSOR_DEBOUNCE_MS = 100UL;
const int LIMIT_ACTIVE_LEVEL = LOW;
const unsigned long PAPER_TRIGGER_DELAY_MS = 500;
const unsigned long MOTOR_RUN_TIME_MS = 1500;
const unsigned long MOTOR_AFTER_SERVO_RETURN_MS = 1500;
const int AUTO_SERVO_PRESS_ANGLE = 141;
const int AUTO_MOTOR_SPEED_PERCENT = 30;
const int AUTO_PUSH_SPEED_PERCENT = -100;
const unsigned long AUTO_PUSH_DURATION_MS = 11000UL;
const int AUTO_HOME_PUSH_SPEED_PERCENT = 100;
const unsigned long AUTO_HOME_PUSH_DURATION_MS = 10000UL;
const int HOME_RELEASE_SPEED_PERCENT = -30;
const unsigned long HOME_RELEASE_TIME_MS = 300UL;
const unsigned long PUSH_TIMEOUT_MAX_MS = 600000UL;
const unsigned long PUSH_RECOVERY_PAUSE_MS = 700UL;
const unsigned long PUSH_RECOVERY_MAX_RUN_MS = 1000UL;
const int PUSH_RECOVERY_SPEED_PERCENT = 30;
const unsigned long PUSH_BACKOFF_TIME_MS = 1500UL;
const unsigned long PUSH_BACKOFF_MAX_RUN_MS = 1500UL;
const unsigned long PUSH_REAPPROACH_PAUSE_MS = 300UL;
const unsigned long PUSH_REAPPROACH_MAX_RUN_MS = 3000UL;
const int PUSH_REAPPROACH_SPEED_PERCENT = 30;
const unsigned long PUSH_FINAL_RELEASE_TIME_MS = 600UL;
const int PUSH_FINAL_RELEASE_SPEED_PERCENT = 30;
const int PUSH_HOMING_SPEED_PERCENT = 20;
const unsigned long LIMIT_DEBOUNCE_MS = 50UL;

enum AutoLoadState {
  AUTO_IDLE,
  AUTO_WAIT_DELAY,
  AUTO_MOVE_SERVO,
  AUTO_RUN_MOTOR,
  AUTO_RETURN_SERVO,
  AUTO_FINAL_MOTOR_DELAY,
  AUTO_WAIT_CLEAR
};

String serialCommand;
int motorSpeedPercent = 0;
int pushSpeedPercent = 0;
int servoCurrentAngle = SERVO_START_ANGLE;
int servoTargetAngle = SERVO_START_ANGLE;
unsigned long lastServoMoveMs = 0;
AutoLoadState autoLoadState = AUTO_IDLE;
unsigned long autoStateStartedMs = 0;
bool lastForwardLimitState = false;
bool lastReverseLimitState = false;
bool rawForwardLimitState = false;
bool rawReverseLimitState = false;
unsigned long rawForwardChangedMs = 0;
unsigned long rawReverseChangedMs = 0;
bool limitStateInitialized = false;
bool timedPushActive = false;
unsigned long timedPushStartedMs = 0;
unsigned long timedPushDurationMs = 0;
bool pushRecoveryPending = false;
bool pushRecoveryActive = false;
unsigned long pushRecoveryStartedMs = 0;
int pushRecoveryPercent = 0;
int pushRecoveryLimitPin = -1;
bool pushHomingActive = false;
bool pushHomingComplete = false;
bool autoPushRunning = false;
bool autoHomePushRunning = false;
bool waitingForHomeCommand = false;
bool homeReleaseActive = false;
unsigned long homeReleaseStartedMs = 0;
bool manualFlowStart = false;
bool autoReapproachSequence = false;
bool pushReapproachPending = false;
bool pushReapproachActive = false;
unsigned long pushReapproachStartedMs = 0;
int pushReapproachPercent = 0;
bool pushFinalReleaseActive = false;
unsigned long pushFinalReleaseStartedMs = 0;
int pushFinalReleasePercent = 0;
bool rawPaperDetected = false;
bool stablePaperDetected = false;
bool paperSensorArmed = false;
bool paperTriggerPending = false;
unsigned long rawPaperChangedMs = 0;
int relayState = 0;
bool relayResetActive = false;
unsigned long relayResetStartedMs = 0;

void setup() {
  Serial.begin(115200);

  setupMotorPwm();
  setMotorSpeed(0);
  pinMode(RELAY_PIN, OUTPUT);
  setRelayState(0);
  setupPushMotorPwm();
  pinMode(LIMIT_FORWARD_PIN, INPUT_PULLUP);
  pinMode(LIMIT_REVERSE_PIN, INPUT_PULLUP);
  setPushMotor(0);
  initializeLimitSwitches();
  pushHomingActive = false;
  pushHomingComplete = true;
  DEBUG_PRINTLN("Push homing skipped");
  pinMode(PAPER_SENSOR_PIN, INPUT);
  initializePaperSensor();
  setupServoPwm();
  servoCurrentAngle = SERVO_START_ANGLE;
  servoTargetAngle = SERVO_START_ANGLE;
  writeServoAngle(servoCurrentAngle);

  DEBUG_PRINTLN("Ready. Nhap help: de xem lenh.");
}

void loop() {
  readSerialCommand();
  updateServoMotion();
  updateLimitSwitches();
  updatePushHoming();
  updateHomeRelease();
  updateTimedPush();
  updatePushRecovery();
  updatePushReapproach();
  updatePushFinalRelease();
  updateRelayReset();
  updatePaperSensor();
  updateAutoLoad();
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
    Serial.println("push:<-100..100>");
    Serial.println("push:0 = stop");
    Serial.println("push_t:<-100..100>,<time_ms> = chay co timeout");
    Serial.println("home = chay ve home +100% toi da 10000 ms");
    Serial.println("begin = kich hoat toan bo flow nhu cam bien giay");
    Serial.println("relay:<0|1>");
    Serial.println("reset = relay 1 trong 500 ms, sau do tat");
    Serial.println("sensor");
    Serial.println("limits");
    Serial.println("stop");
    Serial.println("status");
    Serial.printf("Motor PWM hien tai: %d%%\n", motorSpeedPercent);
    return;
  }

  if (command == "stop") {
    setMotorSpeed(0);
    setPushMotor(0);
    timedPushActive = false;
    pushRecoveryPending = false;
    pushRecoveryActive = false;
    pushHomingActive = false;
    autoPushRunning = false;
    autoHomePushRunning = false;
    waitingForHomeCommand = false;
    homeReleaseActive = false;
    manualFlowStart = false;
    paperTriggerPending = false;
    autoReapproachSequence = false;
    pushReapproachPending = false;
    pushReapproachActive = false;
    pushFinalReleaseActive = false;
    autoLoadState = AUTO_WAIT_CLEAR;
    Serial.println("All motors stopped");
    return;
  }

  if (command == "home") {
    timedPushActive = false;
    pushRecoveryPending = false;
    pushRecoveryActive = false;
    autoPushRunning = false;
    autoHomePushRunning = false;
    waitingForHomeCommand = true;
    startAutoHomePush();
    return;
  }

  if (command == "begin") {
    if (waitingForHomeCommand) {
      Serial.println("e4");
      return;
    }

    bool flowBusy =
        autoLoadState != AUTO_IDLE &&
        autoLoadState != AUTO_WAIT_CLEAR;

    if (flowBusy || timedPushActive ||
        pushRecoveryPending || pushRecoveryActive) {
      Serial.println("e5");
      return;
    }

    manualFlowStart = true;
    paperSensorArmed = false;
    paperTriggerPending = false;
    autoLoadState = AUTO_WAIT_DELAY;
    autoStateStartedMs = millis();
    DEBUG_PRINTLN("Manual begin: waiting 500 ms");
    return;
  }

  if (command == "status") {
    printStatus();
    return;
  }

  if (command == "reset") {
    relayResetActive = true;
    relayResetStartedMs = millis();
    setRelayState(1);
    DEBUG_PRINTLN("Relay reset: ON trong 500 ms");
    return;
  }

  if (command == "relay:0") {
    relayResetActive = false;
    setRelayState(0);
    Serial.println("Relay: 0");
    return;
  }

  if (command == "relay:1") {
    relayResetActive = false;
    setRelayState(1);
    Serial.println("Relay: 1");
    return;
  }

  if (command == "sensor") {
    printSensorStatus();
    return;
  }

  if (command == "limits") {
    printLimitStatus();
    return;
  }

  if (command.startsWith("motor:")) {
    int requestedPercent = command.substring(6).toInt();
    setMotorSpeed(requestedPercent);
    DEBUG_PRINTF("Motor PWM: %d%%\n", motorSpeedPercent);
    return;
  }

  if (command.startsWith("push:")) {
    timedPushActive = false;
    pushRecoveryPending = false;
    pushRecoveryActive = false;
    autoPushRunning = false;
    autoHomePushRunning = false;
    waitingForHomeCommand = false;
    homeReleaseActive = false;
    manualFlowStart = false;
    paperTriggerPending = false;
    autoReapproachSequence = false;
    pushReapproachPending = false;
    pushReapproachActive = false;
    pushFinalReleaseActive = false;
    int requestedPercent = command.substring(5).toInt();
    setPushMotor(requestedPercent);
    DEBUG_PRINTF("Push motor: %d%%\n", pushSpeedPercent);
    return;
  }

  if (command.startsWith("push_t:")) {
    pushRecoveryPending = false;
    pushRecoveryActive = false;
    autoPushRunning = false;
    autoHomePushRunning = false;
    waitingForHomeCommand = false;
    homeReleaseActive = false;
    manualFlowStart = false;
    paperTriggerPending = false;
    autoReapproachSequence = false;
    pushReapproachPending = false;
    pushReapproachActive = false;
    pushFinalReleaseActive = false;
    String parameters = command.substring(7);
    int separator = parameters.indexOf(',');

    if (separator <= 0 || separator >= parameters.length() - 1) {
      Serial.println("e2");
      return;
    }

    int requestedPercent = parameters.substring(0, separator).toInt();
    long requestedDurationMs = parameters.substring(separator + 1).toInt();

    if (requestedDurationMs <= 0 ||
        requestedDurationMs > (long)PUSH_TIMEOUT_MAX_MS) {
      Serial.println("e3");
      return;
    }

    setPushMotor(requestedPercent);

    if (pushSpeedPercent == 0) {
      timedPushActive = false;
      DEBUG_PRINTLN("Push motor stopped; timeout cancelled");
      return;
    }

    timedPushStartedMs = millis();
    timedPushDurationMs = (unsigned long)requestedDurationMs;
    timedPushActive = true;
    DEBUG_PRINTF("Push motor: %d%% for %lu ms\n",
                  pushSpeedPercent, timedPushDurationMs);
    return;
  }

  if (command.startsWith("servo:")) {
    int angle = command.substring(6).toInt();
    servoTargetAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    DEBUG_PRINTF("Servo target: %d degrees, current: %d degrees\n",
                  servoTargetAngle, servoCurrentAngle);
    return;
  }

  Serial.println("e1");
}

void printStatus() {
  Serial.printf("motor_pwm=%d%%, output=%d\n",
                motorSpeedPercent, motorSpeedPercent > 0 ? 1 : 0);
  Serial.printf("push_pwm=%d%%, direction=%s\n",
                abs(pushSpeedPercent),
                pushSpeedPercent > 0 ? "FORWARD" :
                (pushSpeedPercent < 0 ? "REVERSE" : "STOP"));
  Serial.printf("servo_current=%d, servo_target=%d\n",
                servoCurrentAngle, servoTargetAngle);
  Serial.printf("relay=%d\n", relayState);
  printSensorStatus();
  printLimitStatus();
  Serial.printf("auto_state=%s\n", autoLoadStateName());
}

void printLimitStatus() {
  bool forwardLimit = lastForwardLimitState;
  bool reverseLimit = lastReverseLimitState;
  Serial.printf("sensor1=%s, sensor2=%s\n",
                forwardLimit ? "ACTIVE" : "CLEAR",
                reverseLimit ? "ACTIVE" : "CLEAR");
}

void initializeLimitSwitches() {
  rawForwardLimitState = digitalRead(LIMIT_FORWARD_PIN) == LIMIT_ACTIVE_LEVEL;
  rawReverseLimitState = digitalRead(LIMIT_REVERSE_PIN) == LIMIT_ACTIVE_LEVEL;
  lastForwardLimitState = rawForwardLimitState;
  lastReverseLimitState = rawReverseLimitState;
  rawForwardChangedMs = millis();
  rawReverseChangedMs = millis();
  limitStateInitialized = true;

  DEBUG_PRINTF("sensor1 (GPIO19): %s\n",
                lastForwardLimitState ? "ACTIVE" : "CLEAR");
  DEBUG_PRINTF("sensor2 (GPIO18): %s\n",
                lastReverseLimitState ? "ACTIVE" : "CLEAR");
}

void printSensorStatus() {
  int sensorLevel = stablePaperDetected ? PAPER_DETECTED_LEVEL :
                    (PAPER_DETECTED_LEVEL == LOW ? HIGH : LOW);
  bool paperDetected = stablePaperDetected;
  Serial.printf("sensor_do=%d, paper=%s\n",
                sensorLevel, paperDetected ? "YES" : "NO");
}

void initializePaperSensor() {
  rawPaperDetected = digitalRead(PAPER_SENSOR_PIN) == PAPER_DETECTED_LEVEL;
  stablePaperDetected = rawPaperDetected;
  rawPaperChangedMs = millis();
  paperSensorArmed = !stablePaperDetected;

  DEBUG_PRINTF("Paper sensor startup: %s\n",
                stablePaperDetected ?
                "ACTIVE, waiting for CLEAR" : "CLEAR, armed");
}

void updatePaperSensor() {
  unsigned long now = millis();
  bool paperRaw = digitalRead(PAPER_SENSOR_PIN) == PAPER_DETECTED_LEVEL;

  if (paperRaw != rawPaperDetected) {
    rawPaperDetected = paperRaw;
    rawPaperChangedMs = now;
  }

  if (rawPaperDetected != stablePaperDetected &&
      now - rawPaperChangedMs >= PAPER_SENSOR_DEBOUNCE_MS) {
    stablePaperDetected = rawPaperDetected;

    if (stablePaperDetected) {
      if (paperSensorArmed) {
        paperTriggerPending = true;
      }
      paperSensorArmed = false;
      DEBUG_PRINTLN("Paper sensor: ACTIVE");
    } else {
      paperSensorArmed = true;
      DEBUG_PRINTLN("Paper sensor: CLEAR, armed");
    }
  }
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

void updateLimitSwitches() {
  unsigned long now = millis();
  bool forwardRaw = digitalRead(LIMIT_FORWARD_PIN) == LIMIT_ACTIVE_LEVEL;
  bool reverseRaw = digitalRead(LIMIT_REVERSE_PIN) == LIMIT_ACTIVE_LEVEL;

  if (!limitStateInitialized) {
    initializeLimitSwitches();
  }

  if (forwardRaw != rawForwardLimitState) {
    rawForwardLimitState = forwardRaw;
    rawForwardChangedMs = now;
  }

  if (reverseRaw != rawReverseLimitState) {
    rawReverseLimitState = reverseRaw;
    rawReverseChangedMs = now;
  }

  bool homeRawLimitActive =
      autoHomePushRunning &&
      reverseRaw;

  if (homeRawLimitActive) {
    setPushMotor(0);
    timedPushActive = false;
    autoHomePushRunning = false;
    homeReleaseActive = true;
    homeReleaseStartedMs = now;
    setPushMotor(HOME_RELEASE_SPEED_PERCENT);
    DEBUG_PRINTF("Home sensor reached: release %d%% for %lu ms\n",
                  HOME_RELEASE_SPEED_PERCENT, HOME_RELEASE_TIME_MS);
    return;
  }

  if (rawForwardLimitState != lastForwardLimitState &&
      now - rawForwardChangedMs >= LIMIT_DEBOUNCE_MS) {
    lastForwardLimitState = rawForwardLimitState;
    DEBUG_PRINTF("sensor1 (GPIO19): %s\n",
                  lastForwardLimitState ? "ACTIVE" : "CLEAR");
  }

  if (rawReverseLimitState != lastReverseLimitState &&
      now - rawReverseChangedMs >= LIMIT_DEBOUNCE_MS) {
    lastReverseLimitState = rawReverseLimitState;
    DEBUG_PRINTF("sensor2 (GPIO18): %s\n",
                  lastReverseLimitState ? "ACTIVE" : "CLEAR");
  }

  bool forwardLimit = lastForwardLimitState;
  bool reverseLimit = lastReverseLimitState;

  bool directionalLimitActive =
      (pushSpeedPercent > 0 && forwardLimit) ||
      (pushSpeedPercent < 0 && reverseLimit);
  bool anyLimitActiveDuringTimedPush =
      timedPushActive && !autoHomePushRunning &&
      (forwardLimit || reverseLimit);
  bool oppositeLimitActiveDuringRecovery =
      pushRecoveryActive &&
      ((pushSpeedPercent > 0 && reverseLimit) ||
       (pushSpeedPercent < 0 && forwardLimit));

  if ((!pushRecoveryActive && !homeReleaseActive &&
       !autoHomePushRunning &&
       !pushReapproachActive && !pushFinalReleaseActive &&
       directionalLimitActive) ||
      anyLimitActiveDuringTimedPush ||
      oppositeLimitActiveDuringRecovery) {
    int currentPushDirection = pushSpeedPercent;
    setPushMotor(0);

    if (timedPushActive && anyLimitActiveDuringTimedPush) {
      timedPushActive = false;
      autoReapproachSequence = autoPushRunning;
      autoPushRunning = false;
      waitingForHomeCommand = true;
      pushRecoveryPercent = currentPushDirection > 0 ?
                            -PUSH_RECOVERY_SPEED_PERCENT :
                            PUSH_RECOVERY_SPEED_PERCENT;
      pushReapproachPercent = currentPushDirection > 0 ?
                              PUSH_REAPPROACH_SPEED_PERCENT :
                              -PUSH_REAPPROACH_SPEED_PERCENT;
      pushRecoveryLimitPin = forwardLimit ? LIMIT_FORWARD_PIN :
                             (reverseLimit ? LIMIT_REVERSE_PIN : -1);

      pushRecoveryPending = true;
      pushRecoveryStartedMs = millis();
      DEBUG_PRINTF("Triggered sensor: %s\n",
                    forwardLimit ? "sensor1 (GPIO19)" :
                    (reverseLimit ? "sensor2 (GPIO18)" : "unknown"));
      DEBUG_PRINTF("Limit active: motor stopped, recovery in %lu ms\n",
                    PUSH_RECOVERY_PAUSE_MS);
    } else {
      if (autoHomePushRunning && directionalLimitActive) {
        timedPushActive = false;
        autoHomePushRunning = false;
        waitingForHomeCommand = false;
        DEBUG_PRINTLN("Home sensor reached: motor stopped");
      } else {
        DEBUG_PRINTLN("Limit switch active: push motor stopped");
      }
    }
  }
}

void startPushHoming() {
  pushHomingActive = true;
  pushHomingComplete = false;
  setPushMotor(PUSH_HOMING_SPEED_PERCENT);
  DEBUG_PRINTF("Push homing: motor +%d%%, waiting for opto1/GPIO19\n",
                PUSH_HOMING_SPEED_PERCENT);
}

void updatePushHoming() {
  if (!pushHomingActive) {
    return;
  }

  bool homeLimitActive = lastForwardLimitState;

  if (homeLimitActive) {
    setPushMotor(0);
    pushHomingActive = false;
    pushHomingComplete = true;
    DEBUG_PRINTLN("Push homing complete: opto1/GPIO19 reached");
  }
}

void updateHomeRelease() {
  if (!homeReleaseActive) {
    return;
  }

  if (millis() - homeReleaseStartedMs >= HOME_RELEASE_TIME_MS) {
    setPushMotor(0);
    homeReleaseActive = false;
    waitingForHomeCommand = false;
    DEBUG_PRINTLN("Ready: home sensor released");
  }
}

void updateTimedPush() {
  if (!timedPushActive) {
    return;
  }

  if (millis() - timedPushStartedMs >= timedPushDurationMs) {
    setPushMotor(0);
    timedPushActive = false;

    if (autoPushRunning) {
      autoPushRunning = false;
      waitingForHomeCommand = true;
      Serial.println("cut");
      DEBUG_PRINTLN("Auto push complete: waiting for home command");
      return;
    }

    if (autoHomePushRunning) {
      autoHomePushRunning = false;
      waitingForHomeCommand = false;
      DEBUG_PRINTLN("Auto home push complete: motor stopped");
      return;
    }

    DEBUG_PRINTLN("Push timeout: motor stopped");
  }
}

void startAutoHomePush() {
  autoHomePushRunning = true;
  setPushMotor(AUTO_HOME_PUSH_SPEED_PERCENT);

  if (pushSpeedPercent == 0) {
    autoHomePushRunning = false;
    waitingForHomeCommand = false;
    Serial.println("e6");
    return;
  }

  timedPushStartedMs = millis();
  timedPushDurationMs = AUTO_HOME_PUSH_DURATION_MS;
  timedPushActive = true;
  DEBUG_PRINTF("Auto home push: %d%% for %lu ms\n",
                AUTO_HOME_PUSH_SPEED_PERCENT, AUTO_HOME_PUSH_DURATION_MS);
}

void updatePushRecovery() {
  if (pushRecoveryPending &&
      millis() - pushRecoveryStartedMs >= PUSH_RECOVERY_PAUSE_MS) {
    pushRecoveryPending = false;
    pushRecoveryActive = true;
    pushRecoveryStartedMs = millis();
    setPushMotor(pushRecoveryPercent);
    DEBUG_PRINTF("Recovery: push motor %d%% for %lu ms\n",
                  pushRecoveryPercent,
                  autoReapproachSequence ? PUSH_BACKOFF_TIME_MS :
                                           PUSH_RECOVERY_MAX_RUN_MS);
  }

  bool recoveryLimitCleared =
      pushRecoveryLimitPin == LIMIT_FORWARD_PIN ?
        !lastForwardLimitState :
      pushRecoveryLimitPin == LIMIT_REVERSE_PIN ?
        !lastReverseLimitState : true;

  bool recoveryFinished = autoReapproachSequence ?
      ((recoveryLimitCleared &&
        millis() - pushRecoveryStartedMs >= PUSH_BACKOFF_TIME_MS) ||
       millis() - pushRecoveryStartedMs >= PUSH_BACKOFF_MAX_RUN_MS) :
      (recoveryLimitCleared ||
       millis() - pushRecoveryStartedMs >= PUSH_RECOVERY_MAX_RUN_MS);

  if (pushRecoveryActive && recoveryFinished) {
    setPushMotor(0);
    pushRecoveryActive = false;
    pushRecoveryPercent = 0;
    if (autoReapproachSequence) {
      if (!recoveryLimitCleared) {
        autoReapproachSequence = false;
        pushRecoveryLimitPin = -1;
        Serial.println("e9");
      } else {
        pushReapproachPending = true;
        pushReapproachStartedMs = millis();
        DEBUG_PRINTF("Reapproach: waiting %lu ms\n",
                      PUSH_REAPPROACH_PAUSE_MS);
      }
    } else {
      pushRecoveryLimitPin = -1;
      DEBUG_PRINTLN(recoveryLimitCleared ?
                     "Recovery complete: sensor clear, motor stopped" :
                     "e8");
    }

  }
}

void updatePushReapproach() {
  if (pushReapproachPending &&
      millis() - pushReapproachStartedMs >= PUSH_REAPPROACH_PAUSE_MS) {
    pushReapproachPending = false;
    pushReapproachActive = true;
    pushReapproachStartedMs = millis();
    setPushMotor(pushReapproachPercent);
    DEBUG_PRINTF("Reapproach: push motor %d%% until sensor active\n",
                  pushReapproachPercent);
  }

  if (!pushReapproachActive) {
    return;
  }

  bool targetSensorActive =
      pushRecoveryLimitPin == LIMIT_FORWARD_PIN ? lastForwardLimitState :
      pushRecoveryLimitPin == LIMIT_REVERSE_PIN ? lastReverseLimitState : false;

  if (targetSensorActive ||
      millis() - pushReapproachStartedMs >= PUSH_REAPPROACH_MAX_RUN_MS) {
    setPushMotor(0);
    pushReapproachActive = false;

    if (targetSensorActive) {
      pushFinalReleasePercent = pushReapproachPercent > 0 ?
                                -PUSH_FINAL_RELEASE_SPEED_PERCENT :
                                PUSH_FINAL_RELEASE_SPEED_PERCENT;
      pushFinalReleaseActive = true;
      pushFinalReleaseStartedMs = millis();
      setPushMotor(pushFinalReleasePercent);
      DEBUG_PRINTF("Final release: push motor %d%% for %lu ms\n",
                    pushFinalReleasePercent, PUSH_FINAL_RELEASE_TIME_MS);
    } else {
      autoReapproachSequence = false;
      pushRecoveryLimitPin = -1;
      waitingForHomeCommand = true;
      Serial.println("e9");
    }
  }
}

void updatePushFinalRelease() {
  if (!pushFinalReleaseActive) {
    return;
  }

  if (millis() - pushFinalReleaseStartedMs >= PUSH_FINAL_RELEASE_TIME_MS) {
    setPushMotor(0);
    pushFinalReleaseActive = false;
    autoReapproachSequence = false;
    pushRecoveryLimitPin = -1;
    waitingForHomeCommand = true;
    Serial.println("cut");
    DEBUG_PRINTLN("Reapproach complete: limit released");
  }
}

void updateAutoLoad() {
  if (pushHomingActive || waitingForHomeCommand) {
    return;
  }

  bool paperDetected = stablePaperDetected;
  unsigned long now = millis();

  switch (autoLoadState) {
    case AUTO_IDLE:
      if (paperDetected && paperTriggerPending) {
        paperTriggerPending = false;
        paperSensorArmed = false;
        autoLoadState = AUTO_WAIT_DELAY;
        autoStateStartedMs = now;
        DEBUG_PRINTLN("Paper detected: waiting 500 ms");
      }
      break;

    case AUTO_WAIT_DELAY:
      if (now - autoStateStartedMs >= PAPER_TRIGGER_DELAY_MS) {
        manualFlowStart = false;
        servoTargetAngle = AUTO_SERVO_PRESS_ANGLE;
        autoLoadState = AUTO_MOVE_SERVO;
        DEBUG_PRINTF("Servo pressing: target %d degrees\n",
                      AUTO_SERVO_PRESS_ANGLE);
      }
      break;

    case AUTO_MOVE_SERVO:
      if (servoCurrentAngle == AUTO_SERVO_PRESS_ANGLE) {
        setMotorSpeed(AUTO_MOTOR_SPEED_PERCENT);
        autoStateStartedMs = now;
        autoLoadState = AUTO_RUN_MOTOR;
        DEBUG_PRINTF("Motor running: %d%% for %lu ms\n",
                      AUTO_MOTOR_SPEED_PERCENT, MOTOR_RUN_TIME_MS);
      }
      break;

    case AUTO_RUN_MOTOR:
      if (now - autoStateStartedMs >= MOTOR_RUN_TIME_MS) {
        servoTargetAngle = SERVO_START_ANGLE;
        autoLoadState = AUTO_RETURN_SERVO;
        DEBUG_PRINTLN("Returning servo to 90 degrees while motor is running");
      }
      break;

    case AUTO_RETURN_SERVO:
      if (servoCurrentAngle == SERVO_START_ANGLE) {
        autoStateStartedMs = now;
        autoLoadState = AUTO_FINAL_MOTOR_DELAY;
        DEBUG_PRINTF("Servo returned to 90 degrees: motor continues %lu ms\n",
                      MOTOR_AFTER_SERVO_RETURN_MS);
      }
      break;

    case AUTO_FINAL_MOTOR_DELAY:
      if (now - autoStateStartedMs >= MOTOR_AFTER_SERVO_RETURN_MS) {
        setMotorSpeed(0);

        setPushMotor(AUTO_PUSH_SPEED_PERCENT);
        if (pushSpeedPercent != 0) {
          timedPushStartedMs = now;
          timedPushDurationMs = AUTO_PUSH_DURATION_MS;
          timedPushActive = true;
          autoPushRunning = true;
          autoHomePushRunning = false;
          DEBUG_PRINTF("Auto push: %d%% for %lu ms\n",
                        AUTO_PUSH_SPEED_PERCENT, AUTO_PUSH_DURATION_MS);
        } else {
          timedPushActive = false;
          autoPushRunning = false;
          Serial.println("e7");
        }

        autoLoadState = AUTO_WAIT_CLEAR;
        DEBUG_PRINTLN("Auto load complete: main motor stopped");
      }
      break;

    case AUTO_WAIT_CLEAR:
      if (!paperDetected && !timedPushActive &&
          !pushRecoveryPending && !pushRecoveryActive) {
        autoLoadState = AUTO_IDLE;
        DEBUG_PRINTLN("Sensor clear and auto push complete: ready for next paper");
      }
      break;
  }
}

const char* autoLoadStateName() {
  switch (autoLoadState) {
    case AUTO_IDLE: return "IDLE";
    case AUTO_WAIT_DELAY: return "WAIT_DELAY";
    case AUTO_MOVE_SERVO: return "MOVE_SERVO";
    case AUTO_RUN_MOTOR: return "RUN_MOTOR";
    case AUTO_RETURN_SERVO: return "RETURN_SERVO";
    case AUTO_FINAL_MOTOR_DELAY: return "FINAL_MOTOR_DELAY";
    case AUTO_WAIT_CLEAR: return "WAIT_CLEAR";
  }
  return "UNKNOWN";
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

void setRelayState(int state) {
  relayState = state == 1 ? 1 : 0;
  digitalWrite(RELAY_PIN, relayState);
}

void updateRelayReset() {
  if (relayResetActive &&
      millis() - relayResetStartedMs >= RELAY_RESET_TIME_MS) {
    setRelayState(0);
    relayResetActive = false;
    DEBUG_PRINTLN("Relay reset: OFF");
  }
}

void setupPushMotorPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PUSH_RPWM_PIN, PUSH_PWM_FREQUENCY, PUSH_PWM_RESOLUTION);
  ledcAttach(PUSH_LPWM_PIN, PUSH_PWM_FREQUENCY, PUSH_PWM_RESOLUTION);
#else
  ledcSetup(PUSH_RPWM_CHANNEL, PUSH_PWM_FREQUENCY, PUSH_PWM_RESOLUTION);
  ledcSetup(PUSH_LPWM_CHANNEL, PUSH_PWM_FREQUENCY, PUSH_PWM_RESOLUTION);
  ledcAttachPin(PUSH_RPWM_PIN, PUSH_RPWM_CHANNEL);
  ledcAttachPin(PUSH_LPWM_PIN, PUSH_LPWM_CHANNEL);
#endif
}

void setPushMotor(int percent) {
  pushSpeedPercent = constrain(percent, -100, 100);

  bool forwardLimit = lastForwardLimitState;
  bool reverseLimit = lastReverseLimitState;

  if (!pushRecoveryActive && !pushHomingActive &&
      !autoHomePushRunning &&
      !homeReleaseActive && !pushReapproachActive &&
      !pushFinalReleaseActive &&
      ((pushSpeedPercent > 0 && forwardLimit) ||
       (pushSpeedPercent < 0 && reverseLimit))) {
    pushSpeedPercent = 0;
  }

  int duty = map(abs(pushSpeedPercent), 0, 100, 0, 255);
  int rightDuty = pushSpeedPercent > 0 ? duty : 0;
  int leftDuty = pushSpeedPercent < 0 ? duty : 0;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PUSH_RPWM_PIN, rightDuty);
  ledcWrite(PUSH_LPWM_PIN, leftDuty);
#else
  ledcWrite(PUSH_RPWM_CHANNEL, rightDuty);
  ledcWrite(PUSH_LPWM_CHANNEL, leftDuty);
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
