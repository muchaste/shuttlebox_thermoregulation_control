/*
 * Temperature Control Arduino
 * 
 * Dedicated Arduino for controlling heating/cooling relays in shuttlebox experiment
 * Receives commands from PC and controls relays accordingly
 * Temperature monitoring handled by PC via Pico TC-08
 * 
 * Communication Protocol:
 * Commands from PC:
 * - "HEAT_ON" / "HEAT_OFF" - Main heating relay
 * - "COOL_ON" / "COOL_OFF" - Main cooling relay  
 * - "BUFFER_HEAT_ON" / "BUFFER_HEAT_OFF" - Buffer heating pump
 * - "BUFFER_COOL_ON" / "BUFFER_COOL_OFF" - Buffer cooling pump
 * - "ALL_OFF" - Turn off all relays (emergency stop)
 * - "STATUS" - Report current relay states
 * - "RESET" - Reset to all off state
 * 
 * Responses to PC:
 * - "TEMP_CTRL_READY" - Startup message
 * - "HEARTBEAT" - Every 5 seconds
 * - "RELAY_HEAT:1" / "RELAY_HEAT:0" - Heating relay state changes
 * - "RELAY_COOL:1" / "RELAY_COOL:0" - Cooling relay state changes  
 * - "RELAY_BHEAT:1" / "RELAY_BHEAT:0" - Buffer heating state changes
 * - "RELAY_BCOOL:1" / "RELAY_BCOOL:0" - Buffer cooling state changes
 * - "ERROR:message" - Error conditions
 */

#include <avr/wdt.h>

// === Configuration ===
#define HEARTBEAT_INTERVAL 5000 // Heartbeat every 5 seconds
#define RELAY_SETTLE_DELAY 100  // Delay between relay operations to prevent inrush
#define COMMAND_TIMEOUT 30000   // 30 seconds without command = emergency stop

// === Pin Assignments ===
const int heatingRelayPin = 8;
const int coolingRelayPin = 9;
const int bufferHeatingRelayPin = 10;
const int bufferCoolingRelayPin = 11;

// === Safety LED (optional) ===
const int statusLedPin = 13; // Built-in LED for status indication

// === Relay State Variables ===
bool heatingRelay = false;
bool coolingRelay = false;
bool bufferHeatingRelay = false;
bool bufferCoolingRelay = false;

// === Timing Variables ===
unsigned long now;
unsigned long lastHeartbeat = 0;
unsigned long lastCommand = 0;

// === Serial Input Handling ===
String serialBuffer = "";

// === Safety Flags ===
bool emergencyStop = false;
bool safetyOverride = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize relay pins
  pinMode(heatingRelayPin, OUTPUT);
  pinMode(coolingRelayPin, OUTPUT);
  pinMode(bufferHeatingRelayPin, OUTPUT);
  pinMode(bufferCoolingRelayPin, OUTPUT);
  pinMode(statusLedPin, OUTPUT);
  
  // Ensure all relays start OFF
  setAllRelaysOff();
  
  // Initialize timing
  now = millis();
  lastCommand = now;
  
  // Enable watchdog timer (8 seconds)
  wdt_enable(WDTO_8S);
  
  // Startup sequence - blink LED
  for (int i = 0; i < 3; i++) {
    digitalWrite(statusLedPin, HIGH);
    delay(200);
    digitalWrite(statusLedPin, LOW);
    delay(200);
  }
  
  // Send startup message
  Serial.println("TEMP_CTRL_READY");
}

void loop() {
  // Reset watchdog timer
  wdt_reset();
  
  now = millis();
  
  // Handle incoming serial commands
  handleSerialInput();
  
  // Safety monitoring
  performSafetyChecks();
  
  // Update status LED
  updateStatusLed();
  
  // Send heartbeat
  sendHeartbeat();
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
      if (serialBuffer.length() > 64) {
        Serial.println("ERROR:Command too long");
        serialBuffer = "";
      }
    }
  }
}

void processSerialCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  // Update last command time
  lastCommand = now;
  
  // Clear emergency stop if we receive commands
  if (emergencyStop && command != "STATUS") {
    emergencyStop = false;
  }
  
  // Process commands
  if (command == "HEAT_ON") {
    setHeatingRelay(true);
  } else if (command == "HEAT_OFF") {
    setHeatingRelay(false);
  } else if (command == "COOL_ON") {
    setCoolingRelay(true);
  } else if (command == "COOL_OFF") {
    setCoolingRelay(false);
  } else if (command == "BUFFER_HEAT_ON") {
    setBufferHeatingRelay(true);
  } else if (command == "BUFFER_HEAT_OFF") {
    setBufferHeatingRelay(false);
  } else if (command == "BUFFER_COOL_ON") {
    setBufferCoolingRelay(true);
  } else if (command == "BUFFER_COOL_OFF") {
    setBufferCoolingRelay(false);
  } else if (command == "ALL_OFF") {
    emergencyStop = true;
    setAllRelaysOff();
  } else if (command == "STATUS") {
    sendStatusReport();
  } else if (command == "RESET") {
    resetSystem();
  } else if (command == "PING") {
    Serial.println("PONG");
  } else if (command == "SAFETY_OVERRIDE") {
    safetyOverride = true;
  } else if (command == "SAFETY_NORMAL") {
    safetyOverride = false;
  } else {
    Serial.println("ERROR:Unknown command: " + command);
  }
}

void setHeatingRelay(bool state) {
  if (emergencyStop && state) {
    Serial.println("ERROR:Emergency stop active - cannot enable heating");
    return;
  }
  
  if (state && coolingRelay && !safetyOverride) {
    Serial.println("ERROR:Cannot enable heating while cooling active");
    return;
  }
  
  if (state != heatingRelay) {
    heatingRelay = state;
    digitalWrite(heatingRelayPin, state ? HIGH : LOW);
    
    Serial.print("RELAY_HEAT:");
    Serial.println(state ? "1" : "0");
    
    // Small delay to prevent relay contact bounce
    delay(RELAY_SETTLE_DELAY);
  }
}

void setCoolingRelay(bool state) {
  if (emergencyStop && state) {
    Serial.println("ERROR:Emergency stop active - cannot enable cooling");
    return;
  }
  
  if (state && heatingRelay && !safetyOverride) {
    Serial.println("ERROR:Cannot enable cooling while heating active");
    return;
  }
  
  if (state != coolingRelay) {
    coolingRelay = state;
    digitalWrite(coolingRelayPin, state ? HIGH : LOW);
    
    Serial.print("RELAY_COOL:");
    Serial.println(state ? "1" : "0");
    
    delay(RELAY_SETTLE_DELAY);
  }
}

void setBufferHeatingRelay(bool state) {
  if (emergencyStop && state) {
    Serial.println("ERROR:Emergency stop active - cannot enable buffer heating");
    return;
  }
  
  if (state && bufferCoolingRelay && !safetyOverride) {
    Serial.println("ERROR:Cannot enable buffer heating while buffer cooling active");
    return;
  }
  
  if (state != bufferHeatingRelay) {
    bufferHeatingRelay = state;
    digitalWrite(bufferHeatingRelayPin, state ? HIGH : LOW);
    
    Serial.print("RELAY_BHEAT:");
    Serial.println(state ? "1" : "0");
    
    delay(RELAY_SETTLE_DELAY);
  }
}

void setBufferCoolingRelay(bool state) {
  if (emergencyStop && state) {
    Serial.println("ERROR:Emergency stop active - cannot enable buffer cooling");
    return;
  }
  
  if (state && bufferHeatingRelay && !safetyOverride) {
    Serial.println("ERROR:Cannot enable buffer cooling while buffer heating active");
    return;
  }
  
  if (state != bufferCoolingRelay) {
    bufferCoolingRelay = state;
    digitalWrite(bufferCoolingRelayPin, state ? HIGH : LOW);
    
    Serial.print("RELAY_BCOOL:");
    Serial.println(state ? "1" : "0");
    
    delay(RELAY_SETTLE_DELAY);
  }
}

void setAllRelaysOff() {
  digitalWrite(heatingRelayPin, LOW);
  digitalWrite(coolingRelayPin, LOW);
  digitalWrite(bufferHeatingRelayPin, LOW);
  digitalWrite(bufferCoolingRelayPin, LOW);
  
  if (heatingRelay || coolingRelay || bufferHeatingRelay || bufferCoolingRelay) {
    heatingRelay = false;
    coolingRelay = false;
    bufferHeatingRelay = false;
    bufferCoolingRelay = false;
    
    Serial.println("RELAY_HEAT:0");
    Serial.println("RELAY_COOL:0");
    Serial.println("RELAY_BHEAT:0");
    Serial.println("RELAY_BCOOL:0");
  }
}

void performSafetyChecks() {
  // Check for command timeout
  if (now - lastCommand > COMMAND_TIMEOUT) {
    if (!emergencyStop) {
      emergencyStop = true;
      setAllRelaysOff();
      Serial.println("ERROR:Command timeout - emergency stop activated");
    }
  }
}

void updateStatusLed() {
  // LED patterns:
  // Solid ON = Emergency stop
  // Fast blink = Any relay active
  // Slow blink = Normal operation, all off
  // OFF = No communication
  
  if (emergencyStop) {
    digitalWrite(statusLedPin, HIGH);
  } else if (heatingRelay || coolingRelay || bufferHeatingRelay || bufferCoolingRelay) {
    // Fast blink when any relay is active
    digitalWrite(statusLedPin, (now / 250) % 2);
  } else {
    // Slow blink when idle
    digitalWrite(statusLedPin, (now / 1000) % 2);
  }
}

void sendStatusReport() {
  Serial.println("STATUS_START");
  Serial.print("RELAY_HEAT:");
  Serial.println(heatingRelay ? "1" : "0");
  Serial.print("RELAY_COOL:");
  Serial.println(coolingRelay ? "1" : "0");
  Serial.print("RELAY_BHEAT:");
  Serial.println(bufferHeatingRelay ? "1" : "0");
  Serial.print("RELAY_BCOOL:");
  Serial.println(bufferCoolingRelay ? "1" : "0");
  Serial.print("EMERGENCY_STOP:");
  Serial.println(emergencyStop ? "1" : "0");
  Serial.print("SAFETY_OVERRIDE:");
  Serial.println(safetyOverride ? "1" : "0");
  Serial.print("LAST_COMMAND_AGE:");
  Serial.println(now - lastCommand);
  Serial.println("STATUS_END");
}

void resetSystem() {
  // Reset all state variables
  emergencyStop = false;
  safetyOverride = false;
  lastCommand = now;
  
  // Turn off all relays
  setAllRelaysOff();
  
  Serial.println("RESET_COMPLETE");
}

void sendHeartbeat() {
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    Serial.println("HEARTBEAT");
    lastHeartbeat = now;
  }
}