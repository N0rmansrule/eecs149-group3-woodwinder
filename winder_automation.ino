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
const int buttonPin = 12;

// Ratio Logic
const int ratio_num = 5;
const int ratio_den = 1;
long ratio_acc = 0;

// Timing
unsigned long lastStepTime = 0;
const unsigned long stepRate = 1000; // microseconds

#define TRIG_PIN 4
#define ECHO_PIN 5

const int ledPin = 13;

const unsigned long sensorInterval = 1000;
const unsigned long ledInterval = 1000;

unsigned long previousSensorMillis = 0;
unsigned long previousLedMillis = 0;

float distance = 100;
float dist = 0;
bool ledState = LOW;

int val = HIGH;
int prev_val = HIGH;
//int state = 0;

unsigned long lastBtn = 0;

bool buttonPressed(){
  if(digitalRead(buttonPin)==LOW && millis()-lastBtn>200){
    lastBtn = millis();
    return true;
  }
  return false;
  // int buttonState = digitalRead(buttonPin);
  // if(buttonState == LOW){
  //   return true;
  // }else{
  //   return false;
  // }
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
  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;
    dist = getDistance();
  }
  
  switch(state){

    case STOP_FWD:
    Serial.println("STOp FWD MODE");
      if(buttonPressed()){
        digitalWrite(dirPin, HIGH);
        digitalWrite(dirPin2, HIGH);
        state = RUN_FWD;
      }
      break;


    case RUN_FWD:
      Serial.println("RUN FWD mode");
      if(dist >= 9.0){
        state = STOP_BWD;
      }
      break;


    case STOP_BWD:
    Serial.println("STOP BWD MODE");
      if(buttonPressed()){
        digitalWrite(dirPin, LOW);
        digitalWrite(dirPin2, LOW);

        state = RUN_BWD;
      }
      break;


    case RUN_BWD:
    Serial.println("RUN BWD MODE");
      if(dist <= 4.0){
        state = STOP_FWD;
      }
      break;
  }

  if(state == RUN_FWD || state == RUN_BWD){

    unsigned long now = micros();

    if(now - lastStepTime >= stepRate){
      lastStepTime = now;

      stepOnceAandRatio();
    }
  }
    
}
