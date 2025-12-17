#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>

// ---------- NeoPixel ----------
#define NEOPIXEL_PIN 16
#define NUM_PIXELS   1
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Purple
static const uint8_t PURPLE_R = 255;
static const uint8_t PURPLE_G = 0;
static const uint8_t PURPLE_B = 255;

// ---------- UART to UNO ----------
static const int ESP32_TX = 17;   // -> UNO D12 (RX)
static const int ESP32_RX = 27;   // unused
static const uint32_t BAUD = 9600;

// ---------- BLE NUS ----------
static const char* BLE_NAME = "ESP32_BLE_UnoRelay";
static NimBLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID RX_UUID     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // write
static NimBLEUUID TX_UUID     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // notify (unused)

void setPurple(bool on) {
  if (on) strip.setPixelColor(0, strip.Color(PURPLE_R, PURPLE_G, PURPLE_B));
  else    strip.setPixelColor(0, 0);
  strip.show();
}

void startAdvertising() {
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->stop();

  NimBLEAdvertisementData ad;
  ad.setFlags(0x06);
  ad.setName(BLE_NAME);
  ad.addServiceUUID(SERVICE_UUID);

  NimBLEAdvertisementData sr;
  sr.setName(BLE_NAME);

  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  adv->start();

  Serial.println("[BLE] Advertising");
}

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    Serial.println("[BLE] Connected");
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    Serial.println("[BLE] Disconnected");
    startAdvertising();
  }
};

NimBLECharacteristic* txChar = nullptr; // For sending state updates to BLE

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    String cmd = String(v.c_str());
    cmd.trim();
    cmd.toUpperCase();
    
    Serial.print("[BLE RX] ");
    Serial.println(cmd);

    // Handle simple 1/0 commands (backward compatibility)
    if (cmd == "1") {
      // 1 = PAUSE: NeoPixel ON purple, relay ONLY byte '1'
      setPurple(true);
      Serial2.write((uint8_t)'1');
      Serial.println("[RELAY] 1");
    } else if (cmd == "0") {
      // 0 = RESUME: NeoPixel OFF, relay ONLY byte '0'
      setPurple(false);
      Serial2.write((uint8_t)'0');
      Serial.println("[RELAY] 0");
    }
    // Handle new protocol commands - forward to Arduino
    else if (cmd == "START_FORWARD" || cmd == "START_BACK" || 
             cmd == "US_STOP_FOR" || cmd == "US_STOP_BACK" ||
             cmd == "RESET" || cmd == "RESUME" ||
             cmd == "EMERGENCY_STOP" || cmd == "PAUSE") {
      // Forward protocol commands to Arduino
      Serial2.println(cmd.c_str());
      Serial.print("[RELAY] ");
      Serial.println(cmd);
    }
  }
};

void setup() {
  Serial.begin(BAUD);
  Serial2.begin(BAUD, SERIAL_8N1, ESP32_RX, ESP32_TX);

  strip.begin();
  strip.setBrightness(64);
  setPurple(false);

  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService* svc = server->createService(SERVICE_UUID);

  // TX characteristic for sending state updates to BLE clients
  txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rxChar = svc->createCharacteristic(
    RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCB());

  svc->start();
  startAdvertising();

  Serial.print("BLE Name: ");
  Serial.println(BLE_NAME);
  Serial.println("Sends ONLY '1'/'0' bytes to UNO. NeoPixel purple ON for '1'.");
}

void sendToBLE(const char* message) {
  if (txChar != nullptr && txChar->getSubscribedCount() > 0) {
    txChar->setValue((uint8_t*)message, strlen(message));
    txChar->notify();
    Serial.print("[BLE TX] ");
    Serial.println(message);
  }
}

void loop() {
  // Read from Arduino Uno via Serial2
  static String arduinoBuffer = "";
  
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '\n' || c == '\r') {
      // End of message
      if (arduinoBuffer.length() > 0) {
        arduinoBuffer.trim();
        
        Serial.print("[UNO RX] ");
        Serial.println(arduinoBuffer);
        
        // Forward state updates and other messages to BLE
        if (arduinoBuffer.startsWith("STATE:")) {
          // State update - forward to BLE
          sendToBLE(arduinoBuffer.c_str());
        } else if (arduinoBuffer == "CV_STOP") {
          // Computer vision stop signal - forward to BLE
          sendToBLE("CV_STOP");
        } else {
          // Other messages - forward as-is
          sendToBLE(arduinoBuffer.c_str());
        }
        
        arduinoBuffer = "";
      }
    } else if (c >= 32 && c <= 126) {
      // Printable character, add to buffer
      if (arduinoBuffer.length() < 100) {
        arduinoBuffer += c;
      } else {
        // Buffer overflow, reset
        arduinoBuffer = "";
      }
    }
  }
}