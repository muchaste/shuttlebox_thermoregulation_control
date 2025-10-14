/*
 * IR Sensor Monitor Arduino
 * 
 * Dedicated Arduino for monitoring IR beam-break sensors in shuttlebox experiment
 * Reports fish position via serial: 0=passage, 1=left, 2=right
 * 
 * Communication Protocol:
 * - Sends position updates immediately when fish moves
 * - Heartbeat every 5 seconds: "HEARTBEAT"
 * - Status on request: responds to "STATUS" with sensor states
 * - Reset command: responds to "RESET" by clearing all states
 */

#include <avr/wdt.h>

// === Configuration ===
#define DELAY_TIME 2000 // Delay in ms after both barriers are clear before position report
#define HEARTBEAT_INTERVAL 5000 // Heartbeat every 5 seconds
#define SENSOR_UPDATE_INTERVAL_MS 500 // Send sensor states every 1 second

// === Pin Assignments ===
const int leftSensors[] = {2, 3, 4, 5, 6};
const int numLeftSensors = sizeof(leftSensors) / sizeof(leftSensors[0]);
const int rightSensors[] = {A1, A2, A3, A4, A5};
const int numRightSensors = sizeof(rightSensors) / sizeof(rightSensors[0]);

// === Timing Constants ===
const unsigned long debounceDelay = 3;
const unsigned long SENSOR_UPDATE_INTERVAL = 1; // Minimum time between sensor updates

// === Sensor State Variables ===
bool leftSensorStatus[numLeftSensors] = {true, true, true, true, true};
bool rightSensorStatus[numRightSensors] = {true, true, true, true, true};
bool lastLeftStableState[numLeftSensors];
unsigned long leftDebounceStartTime[numLeftSensors];
bool leftDebounceInProgress[numLeftSensors];
bool lastRightStableState[numRightSensors];
unsigned long rightDebounceStartTime[numRightSensors];
bool rightDebounceInProgress[numRightSensors];

// === Position Logic Variables ===
bool leftAllClear = true;
bool rightAllClear = true;
unsigned long delayStartTime = 0;
int lastInterruptionSide = -1; // 0=left, 1=right, -1=none
int lastRestorationSide = -1;  // 0=left, 1=right, -1=none
bool passageReported = false;
bool positionReported = false;
int lastReportedPosition = -1; // -1=none, 0=passage, 1=left, 2=right

// === Timing Variables ===
unsigned long now;
unsigned long lastUpdateTime = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastSensorReport = 0;

// === Serial Input Handling ===
String serialBuffer = "";

void setup() {
  Serial.begin(9600);
  
  // Initialize sensor pins
  for (int i = 0; i < numLeftSensors; i++) {
    pinMode(leftSensors[i], INPUT);
  }
  for (int i = 0; i < numRightSensors; i++) {
    pinMode(rightSensors[i], INPUT);
  }

  // Initialize debounce arrays
  for (int i = 0; i < numLeftSensors; i++) {
    leftDebounceStartTime[i] = 0;
    leftDebounceInProgress[i] = false;
  }
  for (int i = 0; i < numRightSensors; i++) {
    rightDebounceStartTime[i] = 0;
    rightDebounceInProgress[i] = false;
  }

  // Initialize sensor states
  initializeSensorStates();

  // Enable watchdog timer (8 seconds)
  wdt_enable(WDTO_8S);

  // Send startup message
  Serial.println("IR_MONITOR_READY");
  
  // Report initial position if any sensors are blocked
  reportInitialPosition();
}

void loop() {
  // Reset watchdog timer
  wdt_reset();
  
  now = millis();
  
  // Handle incoming serial commands
  handleSerialInput();
  
  // Update sensor states
  updateSensorStates();
  
  // Send periodic sensor state updates
  sendSensorUpdates();
  
  // Send heartbeat
  sendHeartbeat();
}

void initializeSensorStates() {
  // Read initial sensor states
  for (int i = 0; i < numLeftSensors; i++) {
    int state = digitalRead(leftSensors[i]);
    lastLeftStableState[i] = state;
    leftSensorStatus[i] = (state == HIGH);  // HIGH = clear, LOW = interrupted
  }

  for (int i = 0; i < numRightSensors; i++) {
    int state = digitalRead(rightSensors[i]);
    lastRightStableState[i] = state;
    rightSensorStatus[i] = (state == HIGH);
  }
  
  // Update all-clear flags
  updateAllClearFlags();
}

void reportInitialPosition() {
  // Report initial position if any sensor is interrupted
  bool anyBlocked = false;
  for (int i = 0; i < numLeftSensors; i++) {
    if (!leftSensorStatus[i]) anyBlocked = true;
  }
  for (int i = 0; i < numRightSensors; i++) {
    if (!rightSensorStatus[i]) anyBlocked = true;
  }
  
  if (anyBlocked) {
    reportPosition(0); // In passage
  }
}

void updateAllClearFlags() {
  leftAllClear = true;
  rightAllClear = true;
  
  for (int i = 0; i < numLeftSensors; i++) {
    if (!leftSensorStatus[i]) leftAllClear = false;
  }
  for (int i = 0; i < numRightSensors; i++) {
    if (!rightSensorStatus[i]) rightAllClear = false;
  }
}

void updateSensorStates() {
  // Prevent too frequent updates
  if (now - lastUpdateTime < SENSOR_UPDATE_INTERVAL) return;
  lastUpdateTime = now;
  
  bool newLeftAllClear = true;
  bool newRightAllClear = true;
  
  // Process left sensors
  for (int i = 0; i < numLeftSensors; i++) {
    processSensor(leftSensors[i], i, lastLeftStableState[i], 
                  leftDebounceStartTime[i], leftDebounceInProgress[i], 
                  leftSensorStatus[i], 0); // 0 = left side
    if (!leftSensorStatus[i]) newLeftAllClear = false;
  }
  
  // Process right sensors
  for (int i = 0; i < numRightSensors; i++) {
    processSensor(rightSensors[i], i, lastRightStableState[i], 
                  rightDebounceStartTime[i], rightDebounceInProgress[i], 
                  rightSensorStatus[i], 1); // 1 = right side
    if (!rightSensorStatus[i]) newRightAllClear = false;
  }
  
  // Handle position changes
  handlePositionLogic(newLeftAllClear, newRightAllClear);
  
  // Update flags
  leftAllClear = newLeftAllClear;
  rightAllClear = newRightAllClear;
}

void processSensor(int pin, int sensorIndex, bool &lastStableState, 
                   unsigned long &debounceStartTime, bool &debounceInProgress, 
                   bool &sensorStatus, int side) {
  int reading = digitalRead(pin);
  
  if (reading != lastStableState) {
    if (!debounceInProgress) {
      debounceStartTime = now;
      debounceInProgress = true;
    } else if (now - debounceStartTime >= debounceDelay) {
      int confirmRead = digitalRead(pin);
      if (confirmRead != lastStableState) {
        lastStableState = confirmRead;
        debounceInProgress = false;
        
        if (confirmRead == LOW) { // Sensor interrupted
          if (sensorStatus) { // Was clear, now interrupted
            sensorStatus = false;
            lastInterruptionSide = side;
            delayStartTime = 0;
            positionReported = false;
          }
        } else { // Sensor restored
          if (!sensorStatus) { // Was interrupted, now clear
            sensorStatus = true;
            lastRestorationSide = side;
          }
        }
      } else {
        debounceInProgress = false;
      }
    }
  } else {
    debounceInProgress = false;
  }
}

void handlePositionLogic(bool newLeftAllClear, bool newRightAllClear) {
  // Start delay timer when all sensors are clear after interruption
  if (newLeftAllClear && newRightAllClear && lastInterruptionSide != -1 && delayStartTime == 0) {
    delayStartTime = now;
  }
  
  // Report position after delay period
  if (!positionReported && newLeftAllClear && newRightAllClear && delayStartTime > 0) {
    if (now - delayStartTime >= DELAY_TIME) {
      if (lastRestorationSide == 0) {
        reportPosition(1); // Left side
      } else if (lastRestorationSide == 1) {
        reportPosition(2); // Right side
      }
      positionReported = true;
    }
  }
  
  // Report passage (any sensor interrupted)
  if (!newLeftAllClear || !newRightAllClear) {
    if (!passageReported) {
      reportPosition(0); // In passage
      passageReported = true;
      positionReported = false;
    }
  } else {
    passageReported = false;
  }
}

void reportPosition(int position) {
  if (position != lastReportedPosition) {
    Serial.print("POSITION:");
    Serial.println(position);
    lastReportedPosition = position;
  }
}

void sendSensorStates() {
  // Send individual sensor states for visualization
  Serial.print("SENSORS:");
  
  // Send left sensors (5 sensors)
  for (int i = 0; i < numLeftSensors; i++) {
    Serial.print(leftSensorStatus[i] ? "1" : "0");
  }
  
  // Send right sensors (5 sensors)  
  for (int i = 0; i < numRightSensors; i++) {
    Serial.print(rightSensorStatus[i] ? "1" : "0");
  }
  
  Serial.println();
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      // Prevent buffer overflow
      if (serialBuffer.length() > 32) {
        serialBuffer = "";
      }
    }
  }
}

void processSerialCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command == "STATUS") {
    sendStatusReport();
  } else if (command == "RESET") {
    resetSystem();
  } else if (command == "LEFT") {
    // Manual override: simulate fish on left side
    simulatePosition(1);
  } else if (command == "RIGHT") {
    // Manual override: simulate fish on right side  
    simulatePosition(2);
  } else if (command == "PING") {
    Serial.println("PONG");
  } else {
    Serial.println("ERROR:Unknown command");
  }
}

void simulatePosition(int position) {
  lastRestorationSide = (position == 1) ? 0 : 1;
  passageReported = false;
  positionReported = true;
  delayStartTime = now;
  reportPosition(position);
}

void sendStatusReport() {
  Serial.println("STATUS_START");
  
  // Report sensor states
  Serial.print("LEFT_SENSORS:");
  for (int i = 0; i < numLeftSensors; i++) {
    Serial.print(leftSensorStatus[i] ? "1" : "0");
  }
  Serial.println();
  
  Serial.print("RIGHT_SENSORS:");
  for (int i = 0; i < numRightSensors; i++) {
    Serial.print(rightSensorStatus[i] ? "1" : "0");
  }
  Serial.println();
  
  Serial.print("POSITION:");
  Serial.println(lastReportedPosition);
  
  Serial.print("ALL_CLEAR:");
  Serial.print(leftAllClear ? "1" : "0");
  Serial.println(rightAllClear ? "1" : "0");
  
  Serial.println("STATUS_END");
}

void resetSystem() {
  // Reset all state variables
  lastInterruptionSide = -1;
  lastRestorationSide = -1;
  passageReported = false;
  positionReported = false;
  lastReportedPosition = -1;
  delayStartTime = 0;
  
  // Reinitialize sensor states
  initializeSensorStates();
  
  Serial.println("RESET_COMPLETE");
  
  // Report initial position
  reportInitialPosition();
}

void sendSensorUpdates() {
  // Send sensor states periodically for visualization
  if (now - lastSensorReport >= SENSOR_UPDATE_INTERVAL_MS) {
    sendSensorStates();
    lastSensorReport = now;
  }
}

void sendHeartbeat() {
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    Serial.println("HEARTBEAT");
    lastHeartbeat = now;
  }
}