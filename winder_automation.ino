#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial Link(12, 11);

enum State {
  STOP_FWD,
  RUN_FWD,
  STOP_BWD,
  RUN_BWD
};

State state = STOP_FWD;

const int dirPin = 2;
const int stepPin = 3;
const int dirPin2 = 6;
const int stepPin2 = 7;
const int buttonPin = 8;

// Ratio Logic
const int ratio_num = 5;
const int ratio_den = 1;
long ratio_acc = 0;

// Timing
unsigned long lastStepTime = 0;
const unsigned long stepRate = 100; // microseconds (lower = faster, 100 = 4x original speed)

#define TRIG_PIN 4
#define ECHO_PIN 5

const int ledPin = 13;

const unsigned long sensorInterval = 1000;

unsigned long previousSensorMillis = 0;

float distance = 100;
float dist = 0;

unsigned long lastBtn = 0;
unsigned long lastBtnPressTime = 0;
const unsigned long DOUBLE_PRESS_WINDOW = 500; // milliseconds for double press detection

// ESP32 communication
bool paused = false;
bool emergencyStop = false;
bool waitingForReset = false; // Waiting for button press to reset after error
unsigned long lastBTTime = 0;
const unsigned long BT_TIMEOUT = 3000; // Consider BT disconnected after 3 seconds

// Command buffer for receiving strings
String cmdBuffer = "";
const int MAX_CMD_LEN = 50;

// ---------------- LCD helpers ----------------
const char* stateLabel(State s) {
  switch (s) {
    case STOP_FWD: return "STOP_FWD";
    case RUN_FWD:  return "RUN_FWD";
    case STOP_BWD: return "STOP_BWD";
    case RUN_BWD:  return "RUN_BWD";
    default:       return "UNKNOWN";
  }
}

void lcdPrintLine(uint8_t row, String text) {
  if (text.length() > 16) text = text.substring(0, 16);
  while (text.length() < 16) text += ' ';
  lcd.setCursor(0, row);
  lcd.print(text);
}

void updateLCD() {
  static State lastShown = (State)255;
  static float lastDist = -1.0;
  static bool lastBTStatus = false;

  // Check Bluetooth connection status
  bool btConnected = (millis() - lastBTTime) < BT_TIMEOUT;

  // Update state line if state changed or BT status changed
  if (state != lastShown || btConnected != lastBTStatus) {
    String stateStr = String(stateLabel(state));
    String btStr = btConnected ? "true" : "false";
    String line0 = stateStr + " BT:" + btStr;
    lcdPrintLine(0, line0);
    lastShown = state;
    lastBTStatus = btConnected;
  }

  // Update distance line if distance changed (with some threshold to avoid flicker)
  if (abs(dist - lastDist) > 0.1) {
    float dist_mm = dist * 10.0; // Convert cm to mm
    lcdPrintLine(1, String("Dist: ") + String(dist_mm, 1) + " mm");
    lastDist = dist;
  }
}

// Button state tracking for double press detection
static unsigned long firstPressTime = 0;
static bool waitingForSecondPress = false;
static bool pendingSinglePress = false;

bool buttonPressed(){
  unsigned long currentTime = millis();
  
  if(digitalRead(buttonPin)==LOW && millis()-lastBtn>200){
    lastBtn = millis();
    
    if(waitingForSecondPress){
      // Second press detected within window - double press!
      if(currentTime - firstPressTime < DOUBLE_PRESS_WINDOW){
        waitingForSecondPress = false;
        firstPressTime = 0;
        pendingSinglePress = false;
        return false; // Don't treat as single press, it's a double press
      }
    }
    
    // First press or new press after timeout
    firstPressTime = currentTime;
    waitingForSecondPress = true;
    pendingSinglePress = true;
    return false; // Don't return true yet, wait to see if double press
  }
  
  // Check if we've waited too long for second press - then it was a single press
  if(waitingForSecondPress && pendingSinglePress && (currentTime - firstPressTime >= DOUBLE_PRESS_WINDOW)){
    waitingForSecondPress = false;
    firstPressTime = 0;
    bool result = pendingSinglePress;
    pendingSinglePress = false;
    return result; // It was a single press
  }
  
  return false;
}

bool isDoublePress(){
  unsigned long currentTime = millis();
  
  if(digitalRead(buttonPin)==LOW && millis()-lastBtn>200){
    if(waitingForSecondPress && (currentTime - firstPressTime < DOUBLE_PRESS_WINDOW)){
      waitingForSecondPress = false;
      firstPressTime = 0;
      pendingSinglePress = false;
      lastBtn = millis();
      return true;
    }
  }
  
  return false;
}

float getDistance(){
  unsigned long currentMillis = millis();
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void stepOnceAandRatio(){
  // MAIN motor pulse
  digitalWrite(stepPin, HIGH);
  digitalWrite(stepPin, LOW);

  // ACCUMULATE
  ratio_acc += 1;

  while(ratio_acc >= ratio_num){
    // RATIO motor step
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin2, LOW);

    ratio_acc -= ratio_num;
  }
}

void stepOnceAandRatio_linear(){
  // MAIN motor pulse
  digitalWrite(stepPin, HIGH);
  digitalWrite(stepPin, LOW);
}

void sendToESP32(const char* message) {
  Link.println(message);
  Serial.print(F("[TX] "));
  Serial.println(message);
}

void sendStateUpdate() {
  // Send current state to ESP32
  char stateMsg[20];
  snprintf(stateMsg, 20, "STATE:%s", stateLabel(state));
  sendToESP32(stateMsg);
}

void resetToHome() {
  // Reset machine: go backward until distance <= 4.0, then STOP_FWD
  Serial.println(F("-> RESET: Moving backward to home position"));
  digitalWrite(dirPin, LOW);
  state = RUN_BWD;
  waitingForReset = false;
  paused = false;
  emergencyStop = false;
  digitalWrite(ledPin, LOW);
  sendStateUpdate(); // Send state update
  updateLCD();
}

void processCommand(String cmd) {
  cmd.trim(); // Remove whitespace
  cmd.toUpperCase(); // Make case-insensitive
  
  lastBTTime = millis(); // Update BT connection time
  
  Serial.print(F("[RX] "));
  Serial.println(cmd);
  
  if (cmd == "START_FORWARD") {
    if (!emergencyStop && !paused && !waitingForReset && state == STOP_FWD) {
      digitalWrite(dirPin, HIGH);
      digitalWrite(dirPin2, HIGH);
      state = RUN_FWD;
      Serial.println(F("-> RUN_FWD"));
      sendStateUpdate(); // Send state update
      updateLCD();
    }
  }
  else if (cmd == "START_BACK") {
    if (!emergencyStop && !paused && !waitingForReset) {
      digitalWrite(dirPin, LOW);
      state = RUN_BWD;
      Serial.println(F("-> RUN_BWD"));
      sendStateUpdate(); // Send state update
      updateLCD();
    }
  }
  else if (cmd == "US_STOP_FOR") {
    if (state == RUN_FWD) {
      state = STOP_BWD;
      Serial.println(F("-> STOP_BWD (US stop)"));
      sendStateUpdate(); // Send state update
      updateLCD();
    }
  }
  else if (cmd == "US_STOP_BACK") {
    if (state == RUN_BWD) {
      state = STOP_FWD;
      Serial.println(F("-> STOP_FWD (US stop)"));
      sendStateUpdate(); // Send state update
      updateLCD();
    }
  }
  else if (cmd == "RESET" || cmd == "RESUME") {
    // Resume/Reset: reset machine to home position
    if (waitingForReset || paused || emergencyStop) {
      resetToHome(); // resetToHome() already sends state update
    }
  }
  else if (cmd == "EMERGENCY_STOP" || cmd == "PAUSE") {
    // Pause/Error: stop everything and wait for button press or resume
    paused = true;
    waitingForReset = true;
    emergencyStop = true;
    digitalWrite(ledPin, HIGH);
    Serial.println(F("-> PAUSE/ERROR: Waiting for reset"));
    sendStateUpdate(); // Send current state (machine is paused but state remains)
    updateLCD();
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
  }
}

void pollESP32() {
  while (Link.available()) {
    char c = (char)Link.read();
    
    if (c == '\n' || c == '\r') {
      // End of command
      if (cmdBuffer.length() > 0) {
        processCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else if (c >= 32 && c <= 126) {
      // Printable character, add to buffer
      if (cmdBuffer.length() < MAX_CMD_LEN) {
        cmdBuffer += c;
      } else {
        // Buffer overflow, reset
        cmdBuffer = "";
      }
    }
  }
}

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);  // Serial Monitor at 115200 for faster output
  Link.begin(9600);      // ESP32 Bluetooth at 9600

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  updateLCD(); // Initial display

  digitalWrite(dirPin, HIGH);
  digitalWrite(dirPin2, HIGH);
  
  Serial.println(F("=== Wood Winder System ==="));
  Serial.println(F("BT Protocol Ready"));
  Serial.println(F("Commands: START_FORWARD, START_BACK"));
  Serial.println(F("         US_STOP_FOR, US_STOP_BACK"));
  Serial.println(F("         RESET, EMERGENCY_STOP"));
  Serial.println(F("Sends: CV_STOP, STATE:xxx"));
  
  delay(500); // Wait for ESP32 to be ready
  sendStateUpdate(); // Send initial state
}

void loop() {
  pollESP32();

  // Update LCD periodically to check BT status
  static unsigned long lastLCDUpdate = 0;
  if (millis() - lastLCDUpdate >= 500) {
    updateLCD();
    lastLCDUpdate = millis();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;
    dist = getDistance();
    updateLCD(); // Update LCD when distance is read
  }
  
  // Check for double press first - triggers reset in ANY state
  if(isDoublePress()){
    Serial.println(F("Double press detected - RESET"));
    resetToHome();
  }
  
  switch(state){
    case STOP_FWD:
      Serial.println("STOP FWD MODE");
      // If waiting for reset after error, button press triggers reset
      if(buttonPressed() && waitingForReset){
        resetToHome();
      }
      // Normal operation: button press starts forward
      else if(buttonPressed() && !paused && !emergencyStop && !waitingForReset){
        digitalWrite(dirPin, HIGH);
        digitalWrite(dirPin2, HIGH);
        state = RUN_FWD;
        sendStateUpdate(); // Send state update
        updateLCD(); // Update LCD on state change
      }
      break;

    case RUN_FWD:
      Serial.println("RUN FWD mode");
      if(dist >= 10.5){
        state = STOP_BWD;
        sendToESP32("CV_STOP"); // Send computer vision stop signal
        sendStateUpdate(); // Send state update
        updateLCD(); // Update LCD on state change
      }
      // Button press during forward movement triggers reset (if not already waiting)
      if(buttonPressed() && !waitingForReset){
        resetToHome();
      }
      // If waiting for reset, button press also triggers reset
      else if(buttonPressed() && waitingForReset){
        resetToHome();
      }
      break;

    case STOP_BWD:
      Serial.println("STOP BWD MODE");
      // If waiting for reset after error, button press triggers reset
      if(buttonPressed() && waitingForReset){
        resetToHome();
      }
      // Normal operation: button press starts backward
      else if(buttonPressed() && !paused && !emergencyStop && !waitingForReset){
        digitalWrite(dirPin, LOW);
        //digitalWrite(dirPin2, LOW);

        state = RUN_BWD;
        sendStateUpdate(); // Send state update
        updateLCD(); // Update LCD on state change
      }
      break;

    case RUN_BWD:
      Serial.println("RUN BWD MODE");
      // Check if we've reached home position (dist <= 4.0)
      if(dist <= 4.0){
        digitalWrite(dirPin, HIGH);
        digitalWrite(dirPin2, HIGH);
        state = STOP_FWD;
        waitingForReset = false; // Reset complete
        emergencyStop = false;
        paused = false;
        digitalWrite(ledPin, LOW);
        Serial.println(F("-> RESET complete: At home position (STOP_FWD)"));
        sendStateUpdate(); // Send state update
        updateLCD(); // Update LCD on state change
      }
      // Button press during backward movement also triggers reset (redundant but safe)
      if(buttonPressed() && waitingForReset){
        // Already resetting, just continue
      }
      break;
  }

  // Stepping logic: allow backward during reset, but prevent forward when waiting for reset
  if(!paused && !emergencyStop && (state == RUN_FWD || state == RUN_BWD)){
    // Don't step forward if waiting for reset
    if(state == RUN_FWD && waitingForReset){
      // Blocked: waiting for reset
    }
    else {
      unsigned long now = micros();

      if(now - lastStepTime >= stepRate && state == RUN_FWD){
        lastStepTime = now;
        stepOnceAandRatio();
      }else if(now - lastStepTime >= stepRate && state == RUN_BWD){
        lastStepTime = now;
        stepOnceAandRatio_linear();
      }
    }
  }
}
