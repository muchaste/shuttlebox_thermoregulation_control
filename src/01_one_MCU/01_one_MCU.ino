// === Libraries for temperature sensors ===
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#define DELAY_TIME 2000 // Delay in ms after both barriers are clear
// #define SENSOR_DEBUG // Uncomment to enable serial sensor status output

// === Pin Assignments ===
const int leftSensors[] = {2, 3, 4, 5, 6};
const int numLeftSensors = sizeof(leftSensors) / sizeof(leftSensors[0]);
const int rightSensors[] = {A1, A2, A3, A4, A5};
const int numRightSensors = sizeof(rightSensors) / sizeof(rightSensors[0]);

const int heatingRelayPin = 8;
const int coolingRelayPin = 9;
const int bufferHeatingRelayPin = 10;
const int bufferCoolingRelayPin = 11;

#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorLeft  = {0x28, 0xAA, 0x94, 0xAD, 0x16, 0x13, 0x02, 0x8F};
DeviceAddress sensorRight = {0x28, 0xAA, 0x7A, 0xAB, 0x16, 0x13, 0x02, 0xFE};

const float tempDiffThreshold = 2.0;
const float sensorCorrectionOffset = +0.15; // Correction value for both sensors
const unsigned long debounceDelay = 3;
const unsigned long TEMP_INTERVAL = 5000;
const unsigned long RELAY_SETTLE_DELAY = 2000;

// === EEPROM addresses for state persistence ===
const int EEPROM_MAGIC_ADDR = 0;        // Magic number to verify valid data
const int EEPROM_TRIAL_MODE_ADDR = 2;   // Trial mode flag
const int EEPROM_TARGET_SET_ADDR = 3;   // Target temperature set flag
const int EEPROM_TARGET_TEMP_ADDR = 4;  // Target temperature (4 bytes for float)
const int EEPROM_MAGIC_VALUE = 0xABCD;  // Magic number to identify valid EEPROM data

// === Sensor state variables ===
bool leftSensorStatus[numLeftSensors] = {true, true, true, true, true};
bool rightSensorStatus[numRightSensors] = {true, true, true, true, true};
bool lastLeftStableState[numLeftSensors];
unsigned long leftDebounceStartTime[numLeftSensors];
bool leftDebounceInProgress[numLeftSensors];
bool lastRightStableState[numRightSensors];
unsigned long rightDebounceStartTime[numRightSensors];
bool rightDebounceInProgress[numRightSensors];
bool newLeftAllClear, newRightAllClear;

// === Logic variables ===
bool leftAllClear = true;
bool rightAllClear = true;
bool leftRelayActive = false;
bool rightRelayActive = false;
bool tempConversionInProgress = false;
float tempLeft = 0;
float tempRight = 0;
unsigned long now;
unsigned long delayStartTime = 0;
int lastInterruptionSide = -1;
int lastRestorationSide = -1;
unsigned long lastTempRequestTime = 0;
unsigned long lastRelaySwitchTime = 0;
bool lastCooling = false;
bool lastHeating = false;
bool lastPumpCooling = false;
bool lastPumpHeating = false;
float lastTempLeft = -1000;
float lastTempRight = -1000;
bool serialOutputDone = false; 
bool passageReported = false;
int lastPositionState = -1;

// === Acclimation mode ===
bool trialStarted = false;
bool targetSet = false;
float targetTempRight = 0;
float targetTempLeft = 0;
bool positionReported = false; 

// === Serial input handling ===
char serialInput[32];
int serialIndex = 0;

// === Timing variables ===
unsigned long lastUpdateTime = 0; 

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numLeftSensors; i++) pinMode(leftSensors[i], INPUT);
  for (int i = 0; i < numRightSensors; i++) pinMode(rightSensors[i], INPUT);

  pinMode(heatingRelayPin, OUTPUT);
  pinMode(coolingRelayPin, OUTPUT);
  pinMode(bufferHeatingRelayPin, OUTPUT);
  pinMode(bufferCoolingRelayPin, OUTPUT);

  digitalWrite(heatingRelayPin, LOW);
  digitalWrite(coolingRelayPin, LOW);
  digitalWrite(bufferHeatingRelayPin, LOW);
  digitalWrite(bufferCoolingRelayPin, LOW);

  sensors.begin();
  if (!sensors.isConnected(sensorLeft)) Serial.println("Left temperature sensor not found!");
  if (!sensors.isConnected(sensorRight)) Serial.println("Right temperature sensor not found!");
  Serial.println("System ready.");

  // Enable watchdog timer (8 seconds)
  wdt_enable(WDTO_8S);

  // === Load saved state from EEPROM ===
  loadStateFromEEPROM();

  // === Initialize debounce arrays ===
  for (int i = 0; i < numLeftSensors; i++) {
    leftDebounceStartTime[i] = 0;
    leftDebounceInProgress[i] = false;
  }
  
  for (int i = 0; i < numRightSensors; i++) {
    rightDebounceStartTime[i] = 0;
    rightDebounceInProgress[i] = false;
  }

  // === Initialize sensor states ===
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

  // Only print "0" if any sensor is really interrupted
  bool anyBlocked = false;
  for (int i = 0; i < numLeftSensors; i++) {
    if (!leftSensorStatus[i]) anyBlocked = true;
  }
  for (int i = 0; i < numRightSensors; i++) {
    if (!rightSensorStatus[i]) anyBlocked = true;
  }
  if (anyBlocked) {
    Serial.println("0");
    lastPositionState = 0;
  }
}


void loop() {
  // Reset watchdog timer
  wdt_reset();
  
  now = millis();
  updateSensorStates();
  handleSerialInput();
  if (trialStarted) {
    handleTrialLogic();
  } else {
    handleAcclimatizationLogic();
  }
}

// Function for temp measurement with offset
float getCorrectedTemperature(DeviceAddress addr) {
  float rawTemp = sensors.getTempC(addr);
  if (rawTemp == DEVICE_DISCONNECTED_C) return rawTemp;
  return rawTemp + sensorCorrectionOffset;
}

// === EEPROM State Management ===
void saveStateToEEPROM() {
  // Write magic number to indicate valid data
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  
  // Save mode states
  EEPROM.put(EEPROM_TRIAL_MODE_ADDR, trialStarted);
  EEPROM.put(EEPROM_TARGET_SET_ADDR, targetSet);
  
  // Save target temperature
  EEPROM.put(EEPROM_TARGET_TEMP_ADDR, targetTempRight);
  
  Serial.println("State saved to EEPROM");
}

void loadStateFromEEPROM() {
  int magicCheck;
  EEPROM.get(EEPROM_MAGIC_ADDR, magicCheck);
  
  // Check if we have valid saved data
  if (magicCheck == EEPROM_MAGIC_VALUE) {
    // Restore mode states
    EEPROM.get(EEPROM_TRIAL_MODE_ADDR, trialStarted);
    EEPROM.get(EEPROM_TARGET_SET_ADDR, targetSet);
    
    // Restore target temperature
    EEPROM.get(EEPROM_TARGET_TEMP_ADDR, targetTempRight);
    targetTempLeft = targetTempRight - tempDiffThreshold;
    
    // Print restoration message
    if (trialStarted) {
      Serial.println("State restored: Trial mode active");
    } else if (targetSet) {
      Serial.print("State restored: Acclimation mode at ");
      Serial.print(targetTempRight);
      Serial.println("°C");
    } else {
      Serial.println("State restored: Waiting for commands");
    }
  } else {
    Serial.println("No previous state found, starting fresh");
  }
}

void clearEEPROMState() {
  // Clear magic number to invalidate saved data
  EEPROM.put(EEPROM_MAGIC_ADDR, 0);
  Serial.println("EEPROM state cleared");
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialInput[serialIndex] = '\0';

      // --- Existing commands ---
      if (serialInput[0] == 't') {
        targetTempRight = atof(serialInput + 1);
        targetTempLeft = targetTempRight - tempDiffThreshold;
        targetSet = true;
        trialStarted = false;  // Entering acclimation mode
        Serial.print("Target temperature set to: ");
        Serial.println(targetTempRight);
        saveStateToEEPROM();  // Save state after setting target
      } else if (strcmp(serialInput, "start") == 0) {
        trialStarted = true;
        positionReported = false;
        Serial.println("Trial mode started.");
        saveStateToEEPROM();  // Save state after starting trial
      } else if (strcmp(serialInput, "reset") == 0) {
        // Reset to initial state
        trialStarted = false;
        targetSet = false;
        targetTempRight = 0;
        targetTempLeft = 0;
        clearEEPROMState();
        Serial.println("System reset to initial state.");
      }
      // --- NEW: manual side input commands ---
      else if (strcmp(serialInput, "left") == 0) {
      // Treat this as if the sensors were just restored on the left side
        lastRestorationSide = 0;      // 0 = left
        passageReported = false;      // no longer in passage
        positionReported = true;      // report position now
        lastPositionState = 1;        // optional: record current position (1 = left)
        delayStartTime = now;         // start the DELAY_TIME countdown (so relays follow your existing delay logic)
        Serial.println("1");          // immediate confirmation (position = left)
      }
      else if (strcmp(serialInput, "right") == 0) {
        // Treat this as if the sensors were just restored on the right side
        lastRestorationSide = 1;      // 1 = right
        passageReported = false;
        positionReported = true;
        lastPositionState = 2;        // optional: record current position (2 = right)
        delayStartTime = now;
        Serial.println("2");          // immediate confirmation (position = right)
      }

      serialIndex = 0;
    } else if (serialIndex < 31) {
      serialInput[serialIndex++] = c;
    }
  }
}

void updateSensorStates() {
  // Add timeout protection
  if (now - lastUpdateTime < 1) return; // Prevent too frequent updates
  lastUpdateTime = now;
  
  newLeftAllClear = true;
  newRightAllClear = true;
  for (int i = 0; i < numLeftSensors; i++) {
    int reading = digitalRead(leftSensors[i]);
    if (reading != lastLeftStableState[i]) {
      if (!leftDebounceInProgress[i]) {
        leftDebounceStartTime[i] = now;
        leftDebounceInProgress[i] = true;
      } else if (now - leftDebounceStartTime[i] >= debounceDelay) {
        int confirmRead = digitalRead(leftSensors[i]);
        if (confirmRead != lastLeftStableState[i]) {
          lastLeftStableState[i] = confirmRead;
          leftDebounceInProgress[i] = false;
          if (confirmRead == LOW) {
            newLeftAllClear = false;
            if (leftSensorStatus[i]) {
              leftSensorStatus[i] = false;
              lastInterruptionSide = 0;
              delayStartTime = 0;
              serialOutputDone = false;
              positionReported = false;
              #ifdef SENSOR_DEBUG
              Serial.print("Sensor L"); Serial.print(i + 1); Serial.println(" interrupted!");
              #endif
            }
          } else if (!leftSensorStatus[i]) {
            leftSensorStatus[i] = true;
            lastRestorationSide = 0;
            #ifdef SENSOR_DEBUG
            Serial.print("Sensor L"); Serial.print(i + 1); Serial.println(" restored!");
            #endif
          }

        } else leftDebounceInProgress[i] = false;
      }
    } else leftDebounceInProgress[i] = false;
    if (lastLeftStableState[i] == LOW) newLeftAllClear = false;
  }
  for (int i = 0; i < numRightSensors; i++) {
    int reading = digitalRead(rightSensors[i]);
    if (reading != lastRightStableState[i]) {
      if (!rightDebounceInProgress[i]) {
        rightDebounceStartTime[i] = now;
        rightDebounceInProgress[i] = true;
      } else if (now - rightDebounceStartTime[i] >= debounceDelay) {
        int confirmRead = digitalRead(rightSensors[i]);
        if (confirmRead != lastRightStableState[i]) {
          lastRightStableState[i] = confirmRead;
          rightDebounceInProgress[i] = false;
          if (confirmRead == LOW) {
            newRightAllClear = false;
            if (rightSensorStatus[i]) {
              rightSensorStatus[i] = false;
              lastInterruptionSide = 1;
              delayStartTime = 0;
              serialOutputDone = false;
              positionReported = false;
              #ifdef SENSOR_DEBUG
              Serial.print("Sensor R"); Serial.print(i + 1); Serial.println(" interrupted!");
              #endif
            }
          } else if (!rightSensorStatus[i]) {
            rightSensorStatus[i] = true;
            lastRestorationSide = 1;
            #ifdef SENSOR_DEBUG
            Serial.print("Sensor R"); Serial.print(i + 1); Serial.println(" restored!");
            #endif
          }

        } else rightDebounceInProgress[i] = false;
      }
    } else rightDebounceInProgress[i] = false;
    if (lastRightStableState[i] == LOW) newRightAllClear = false;
  }
  if (newLeftAllClear && newRightAllClear && lastInterruptionSide != -1 && delayStartTime == 0) {
    delayStartTime = now;
  }
  if (!positionReported && newLeftAllClear && newRightAllClear) {
    if (lastRestorationSide == 0) {
      Serial.println("1");
      positionReported = true;
    } else if (lastRestorationSide == 1) {
      Serial.println("2");
      positionReported = true;
    }
  }
  // Fish is in the passage (any sensor interrupted)
  if (!newLeftAllClear || !newRightAllClear) {
    if (!passageReported) {
      Serial.println("0");
      passageReported = true;
      positionReported = false;
    }
  } else {
    passageReported = false;
  }
  leftAllClear = newLeftAllClear;
  rightAllClear = newRightAllClear;
}

// Acclimation logic with independent cooling/warming + buffer pumps for temp diff maintenance
void handleAcclimatizationLogic() {
  if (!targetSet) return;

  // Actualize temperatures regularly
  if (now - lastTempRequestTime >= TEMP_INTERVAL) {
    sensors.requestTemperatures();
    
    // Add timeout for sensor response
    unsigned long sensorStart = millis();
    while (!sensors.isConversionComplete() && (millis() - sensorStart < 1000)) {
      delay(10);
    }
    
    if (millis() - sensorStart >= 1000) {
      Serial.println("Temperature sensor timeout!");
      return;
    }
    
    tempLeft = getCorrectedTemperature(sensorLeft);
    tempRight = getCorrectedTemperature(sensorRight);
    lastTempRequestTime = now;
  }

  // Heat right side when below set temp
  if (tempRight < targetTempRight) {
    digitalWrite(heatingRelayPin, HIGH);
  } else if (tempRight >= targetTempRight) {
    digitalWrite(heatingRelayPin, LOW);
  }

  // Cool left side when above set temp
  if (tempLeft > targetTempLeft) {
    digitalWrite(coolingRelayPin, HIGH);
  } else if (tempLeft <= targetTempLeft) {
    digitalWrite(coolingRelayPin, LOW);
  }

  // Activate buffer pumps when temp diff exceeds 2°C
  float diff = abs(tempLeft - tempRight);
  if (diff > tempDiffThreshold) {
    digitalWrite(bufferHeatingRelayPin, HIGH);
    digitalWrite(bufferCoolingRelayPin, HIGH);
  } else if (diff <= tempDiffThreshold) {
    digitalWrite(bufferHeatingRelayPin, LOW);
    digitalWrite(bufferCoolingRelayPin, LOW);
  }
}

void handleTrialLogic() {
  if (delayStartTime > 0 && now - delayStartTime >= DELAY_TIME) {
    if (lastRestorationSide == 0 && !leftRelayActive && newLeftAllClear) {
      digitalWrite(coolingRelayPin, HIGH);
      digitalWrite(heatingRelayPin, LOW);
      leftRelayActive = true;
      rightRelayActive = false;
      lastRelaySwitchTime = now;
      Serial.println("3");
    } else if (lastRestorationSide == 1 && !rightRelayActive && newRightAllClear) {
      digitalWrite(heatingRelayPin, HIGH);
      digitalWrite(coolingRelayPin, LOW);
      rightRelayActive = true;
      leftRelayActive = false;
      lastRelaySwitchTime = now;
      Serial.println("4");
    }
  }

  if (!tempConversionInProgress && now - lastTempRequestTime >= TEMP_INTERVAL && now - lastRelaySwitchTime >= RELAY_SETTLE_DELAY) {
    sensors.requestTemperatures();
    tempConversionInProgress = true;
    lastTempRequestTime = now;
  }

  if (tempConversionInProgress && now - lastTempRequestTime >= 800) {
    // Add timeout for sensor response
    unsigned long sensorStart = millis();
    while (!sensors.isConversionComplete() && (millis() - sensorStart < 500)) {
      delay(10);
    }
    
    tempLeft = getCorrectedTemperature(sensorLeft);
    tempRight = getCorrectedTemperature(sensorRight);
    tempConversionInProgress = false;
    
    if (tempLeft == -127.0 || tempRight == -127.0 || tempLeft == DEVICE_DISCONNECTED_C || tempRight == DEVICE_DISCONNECTED_C) {
      Serial.println("Temperature read error!");
    } else {
      float tempDiff = abs(tempLeft - tempRight);
      bool cooling = digitalRead(coolingRelayPin) == HIGH;
      bool heating = digitalRead(heatingRelayPin) == HIGH;

      digitalWrite(bufferCoolingRelayPin, (cooling && !heating) ? HIGH : LOW);
      digitalWrite(bufferHeatingRelayPin, (heating && !cooling) ? HIGH : LOW);

      bool pumpCooling = cooling && tempDiff > tempDiffThreshold;
      bool pumpHeating = heating && tempDiff > tempDiffThreshold;

      digitalWrite(bufferCoolingRelayPin, pumpCooling ? HIGH : LOW);
      digitalWrite(bufferHeatingRelayPin, pumpHeating ? HIGH : LOW);

      lastCooling = cooling;
      lastHeating = heating;
      lastPumpCooling = pumpCooling;
      lastPumpHeating = pumpHeating;
      lastTempLeft = tempLeft;
      lastTempRight = tempRight;
    }
  }
}
