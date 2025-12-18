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

// ---------- UART to UNO (SoftwareSerial on UNO: Link(12, 11)) ----------
static const int ESP32_TX = 17;   // -> UNO D12 (RX)
static const int ESP32_RX = 27;   // <- UNO D11 (TX)  (level shift UNO->ESP32!)
static const uint32_t LINK_BAUD = 9600;

// ---------- BLE NUS ----------
static const char* BLE_NAME = "ESP32_BLE_UnoRelay";
static NimBLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID RX_UUID     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // write
static NimBLEUUID TX_UUID     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // notify

static volatile bool bleConnected = false;
NimBLECharacteristic* txChar = nullptr;

// -------------------- Helpers --------------------
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

void sendToBLE(const char* message) {
  if (txChar == nullptr) return;
  if (!bleConnected) return;

  txChar->setValue((uint8_t*)message, strlen(message));
  txChar->notify();

  Serial.print("[BLE TX] ");
  Serial.println(message);
}

static void sendBleStatusToUNO() {
  // Line-based so UNO can parse easily
  Serial2.println(bleConnected ? "BLE:1" : "BLE:0");
}

// Relay function used by BOTH BLE and ESP32 Serial Monitor input
void relayToUNO(const String& incoming, const char* sourceTag) {
  String raw = incoming;
  raw.trim();
  if (raw.length() == 0) return;

  String upper = raw;
  upper.toUpperCase();

  Serial.print(sourceTag);
  Serial.print(" ");
  Serial.println(raw);

  // Special 1/0 behavior (single byte, plus NeoPixel)
  if (upper == "1") {
    setPurple(true);
    Serial2.write((uint8_t)'1');   // single byte
    Serial.println("[RELAY->UNO] single byte: '1'");
    return;
  }
  if (upper == "0") {
    setPurple(false);
    Serial2.write((uint8_t)'0');   // single byte
    Serial.println("[RELAY->UNO] single byte: '0'");
    return;
  }

  // Commands (line-based) - includes PAUSE/RESUME
  if (upper == "START_FORWARD" || upper == "START_BACK" ||
      upper == "US_STOP_FOR"   || upper == "US_STOP_BACK" ||
      upper == "RESET"         || upper == "RESUME" ||
      upper == "EMERGENCY_STOP"|| upper == "PAUSE") {
    Serial2.println(upper);
    Serial.print("[RELAY->UNO] ");
    Serial.println(upper);
    return;
  }

  // Anything else line-based
  Serial2.println(raw);
  Serial.print("[RELAY->UNO] ");
  Serial.println(raw);
}

// -------------------- BLE Callbacks --------------------
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    bleConnected = true;
    Serial.println("[BLE] Connected");
    sendBleStatusToUNO();
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    bleConnected = false;
    Serial.println("[BLE] Disconnected");
    sendBleStatusToUNO();
    startAdvertising();
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue();
    if (v.empty()) return;
    relayToUNO(String(v.c_str()), "[BLE RX]");
  }
};

void setup() {
  Serial.begin(115200);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, ESP32_RX, ESP32_TX);

  strip.begin();
  strip.setBrightness(64);
  setPurple(false);

  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService* svc = server->createService(SERVICE_UUID);

  txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rxChar = svc->createCharacteristic(
    RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCB());

  svc->start();
  startAdvertising();

  // ensure UNO starts with correct status
  bleConnected = false;
  sendBleStatusToUNO();

  Serial.print("BLE Name: ");
  Serial.println(BLE_NAME);
  Serial.println("ESP32 Serial Monitor OR BLE -> forwarded to UNO.");
  Serial.println("Commands: PAUSE / RESUME / 1 / 0 / etc.");
}

void loop() {
  // 1) ESP32 USB Serial -> UNO (line-based)
  static String usbBuffer;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbBuffer.length() > 0) {
        relayToUNO(usbBuffer, "[USB RX]");
        usbBuffer = "";
      }
    } else if (c >= 32 && c <= 126) {
      if (usbBuffer.length() < 200) usbBuffer += c;
      else usbBuffer = "";
    }
  }

  // 2) UNO -> ESP32 -> print + forward to BLE (line-based)
  static String unoBuffer;
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    if (c == '\n' || c == '\r') {
      if (unoBuffer.length() > 0) {
        unoBuffer.trim();

        // If UNO tells us 1/0, reflect it on NeoPixel too
        if (unoBuffer == "1") setPurple(true);
        else if (unoBuffer == "0") setPurple(false);

        Serial.print("[UNO RX] ");
        Serial.println(unoBuffer);

        sendToBLE(unoBuffer.c_str());
        unoBuffer = "";
      }
    } else if (c >= 32 && c <= 126) {
      if (unoBuffer.length() < 200) unoBuffer += c;
      else unoBuffer = "";
    }
  }
}
