const int dirPin = 2;
const int stepPin = 3;
const int dirPin2 = 6;
const int stepPin2 = 7;
const int buttonPin = 13;

#define TRIG_PIN 4
#define ECHO_PIN 5

const int ledPin = 13;

const unsigned long sensorInterval = 1000;
const unsigned long ledInterval = 1000;

unsigned long previousSensorMillis = 0;
unsigned long previousLedMillis = 0;

float distance = 100;
bool ledState = LOW;

int val = HIGH;
int prev_val = HIGH;
int state = 0;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);

  digitalWrite(dirPin, HIGH);
  digitalWrite(dirPin2, HIGH);
}

void loop() {
  unsigned long currentMillis = millis();

  // -------- BUTTON TOGGLE ----------
  val = digitalRead(buttonPin);
  if (prev_val == LOW && val == HIGH) {  
    state = !state;  
		Serial.println("Pressed");   
    delay(50);          // debounce
  }
  prev_val = val;

  // -------- LED BLINK -------------
  if (currentMillis - previousLedMillis >= ledInterval) {
    previousLedMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }

  // -------- ULTRASONIC ------------
  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // -------- STEPPER CONTROL --------
	//Button or Utlra Sonic
  if (state == 1) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
}
