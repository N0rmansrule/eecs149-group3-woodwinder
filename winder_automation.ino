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

// ESP32 pause/resume
bool paused = false;
char lastCmd = '\0';
unsigned long lastBTTime = 0;
const unsigned long BT_TIMEOUT = 3000; // Consider BT disconnected after 3 seconds

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

bool buttonPressed(){
  if(digitalRead(buttonPin)==LOW && millis()-lastBtn>200){
    lastBtn = millis();
    return true;
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

void pollESP32() {
  while (Link.available()) {
    char c = (char)Link.read();
    lastBTTime = millis(); // Update BT connection time
    
    if (c == '1') {
      if (!paused) {
        paused = true;
        digitalWrite(ledPin, HIGH);
        Serial.println(F("PAUSE"));
        updateLCD(); // Update to show BT status
      }
    } else if (c == '0') {
      if (paused) {
        paused = false;
        digitalWrite(ledPin, LOW);
        Serial.println(F("RESUME"));
        updateLCD(); // Update to show BT status
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
  
  Serial.println(F("Ready"));
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
  
  switch(state){
    case STOP_FWD:
      Serial.println("STOP FWD MODE");
      if(buttonPressed() && !paused){
        digitalWrite(dirPin, HIGH);
        digitalWrite(dirPin2, HIGH);
        state = RUN_FWD;
        updateLCD(); // Update LCD on state change
      }
      break;

    case RUN_FWD:
      Serial.println("RUN FWD mode");
      if(dist >= 10.5){
        state = STOP_BWD;
        updateLCD(); // Update LCD on state change
      }
      break;

    case STOP_BWD:
      Serial.println("STOP BWD MODE");
      if(buttonPressed() && !paused){
        digitalWrite(dirPin, LOW);
        //digitalWrite(dirPin2, LOW);

        state = RUN_BWD;
        updateLCD(); // Update LCD on state change
      }
      break;

    case RUN_BWD:
      Serial.println("RUN BWD MODE");
      if(dist <= 4.0){
        state = STOP_FWD;
        updateLCD(); // Update LCD on state change
      }
      break;
  }

  if(!paused && (state == RUN_FWD || state == RUN_BWD)){
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
