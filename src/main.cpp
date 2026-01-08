/**
 * ESP32-S3 LoRa Mesh with BLE Web Interface
 *
 * Hardware: ESP32-S3FN8 + SX1262 LoRa (863-928MHz)
 *
 * Features:
 * - BLE connection to Chrome Web Bluetooth API
 * - Parent/Child device architecture
 * - LoRa mesh messaging between devices
 * - Message relay and device management
 */

#include <Arduino.h>
#include <NimBLEDevice.h>

#ifndef WOKWI_SIMULATION
#include <RadioLib.h>
#endif

// ================= PIN DEFINITIONS =================
#define LED_STATUS    2     // Status LED (active tasks)
#define LED_BLE       4     // BLE connection LED
#define BTN_MODE      15    // Mode button

// SX1262 LoRa pins (defined in platformio.ini or use defaults)
#ifndef LORA_CS
#define LORA_CS       8
#endif
#ifndef LORA_DIO1
#define LORA_DIO1     14
#endif
#ifndef LORA_RST
#define LORA_RST      12
#endif
#ifndef LORA_BUSY
#define LORA_BUSY     13
#endif
#ifndef LORA_FREQ
#define LORA_FREQ     915.0   // MHz (use 868.0 for EU)
#endif

// LoRa parameters
#define LORA_BW       125.0   // kHz
#define LORA_SF       9       // Spreading factor (7-12)
#define LORA_CR       7       // Coding rate (5-8)
#define LORA_SW       0x12    // Sync word
#define LORA_POWER    14      // dBm (max 22)
#define LORA_PREAMBLE 8       // Preamble length

// ================= BLE UUIDs =================
#define SERVICE_UUID           "12345678-1234-1234-1234-123456789abc"
#define CHAR_TX_UUID           "12345678-1234-1234-1234-123456789abd"
#define CHAR_RX_UUID           "12345678-1234-1234-1234-123456789abe"
#define CHAR_DEVICE_INFO_UUID  "12345678-1234-1234-1234-123456789abf"

// ================= MESH PROTOCOL =================
#define MESH_MAX_HOPS     5
#define MESH_MSG_TTL      30000   // Message TTL in ms
#define MESH_MAX_SEEN     32      // Max messages to track

// Message types
enum MeshMsgType : uint8_t {
  MSG_BROADCAST = 0x01,    // Broadcast to all
  MSG_DIRECT    = 0x02,    // Direct to specific device
  MSG_DISCOVERY = 0x03,    // Device discovery
  MSG_ACK       = 0x04,    // Acknowledgment
  MSG_HEARTBEAT = 0x05,    // Periodic heartbeat
  MSG_DATA      = 0x10,    // User data message
};

// Mesh message structure (packed for transmission)
#pragma pack(push, 1)
struct MeshMessage {
  uint8_t  type;           // Message type
  uint8_t  hops;           // Hop count
  uint16_t msgId;          // Message ID
  char     srcId[6];       // Source device (last 6 chars of MAC)
  char     dstId[6];       // Destination ("*" for broadcast)
  uint8_t  dataLen;        // Data length
  char     data[200];      // Payload
};
#pragma pack(pop)

// ================= DEVICE CONFIGURATION =================
enum DeviceRole {
  ROLE_STANDALONE,
  ROLE_PARENT,
  ROLE_CHILD
};

struct DeviceConfig {
  char deviceId[13];       // Full MAC-based ID
  char shortId[7];         // Last 6 chars for mesh
  char deviceName[32];
  DeviceRole role;
  bool bleConnected;
  int8_t lastRssi;
  float lastSnr;
};

DeviceConfig config;

// Track seen messages to prevent loops
struct SeenMessage {
  uint16_t msgId;
  char srcId[7];
  unsigned long timestamp;
};
SeenMessage seenMessages[MESH_MAX_SEEN];
uint8_t seenIndex = 0;

// Track discovered devices
#define MAX_DEVICES 16
struct DiscoveredDevice {
  char id[7];
  char name[32];
  int8_t rssi;
  unsigned long lastSeen;
};
DiscoveredDevice devices[MAX_DEVICES];
uint8_t deviceCount = 0;

// ================= LORA OBJECTS =================
#ifndef WOKWI_SIMULATION
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
volatile bool loraReceived = false;
volatile bool loraTransmitting = false;

// ISR for receive
void IRAM_ATTR loraISR() {
  loraReceived = true;
}
#endif

// ================= BLE OBJECTS =================
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pTxCharacteristic = nullptr;
NimBLECharacteristic* pRxCharacteristic = nullptr;
NimBLECharacteristic* pDeviceInfoCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ================= MESSAGE BUFFER =================
#define MSG_BUFFER_SIZE 512
char rxBuffer[MSG_BUFFER_SIZE];
volatile bool newBleMessage = false;

// Message ID counter
uint16_t msgIdCounter = 0;

// ================= FUNCTION PROTOTYPES =================
void initLoRa();
void handleLoRaReceive();
void sendLoRaMessage(MeshMsgType type, const char* destId, const char* data, uint8_t dataLen);
void broadcastDiscovery();
void sendHeartbeat();
void relayMessage(MeshMessage* msg);
bool isMessageSeen(uint16_t msgId, const char* srcId);
void markMessageSeen(uint16_t msgId, const char* srcId);
void updateDeviceList(const char* id, const char* name, int8_t rssi);
void sendBleMessage(const char* message);
void handleBleCommand(const char* message);
void updateDeviceInfo();

// ================= BLE CALLBACKS =================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
    deviceConnected = true;
    config.bleConnected = true;
    digitalWrite(LED_BLE, HIGH);
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(NimBLEServer* pServer) {
    deviceConnected = false;
    config.bleConnected = false;
    digitalWrite(LED_BLE, LOW);
    Serial.println("[BLE] Client disconnected");
    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      strncpy(rxBuffer, value.c_str(), MSG_BUFFER_SIZE - 1);
      rxBuffer[MSG_BUFFER_SIZE - 1] = '\0';
      newBleMessage = true;
      Serial.printf("[BLE] Received: %s\n", rxBuffer);
    }
  }
};

// ================= HELPER FUNCTIONS =================
void generateDeviceId() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  snprintf(config.deviceId, sizeof(config.deviceId),
           "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // Short ID is last 6 chars
  strncpy(config.shortId, config.deviceId + 6, 6);
  config.shortId[6] = '\0';
}

void blinkLED(int pin, int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}

// ================= LORA FUNCTIONS =================
#ifndef WOKWI_SIMULATION
void initLoRa() {
  Serial.println("[LoRa] Initializing SX1262...");

  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SW, LORA_POWER, LORA_PREAMBLE);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] Initialized successfully!");
  } else {
    Serial.printf("[LoRa] Init failed, code: %d\n", state);
    return;
  }

  // Configure for low power, good range
  radio.setCurrentLimit(60.0);
  radio.setDio2AsRfSwitch(true);
  radio.setCRC(true);

  // Set up interrupt for receive
  radio.setDio1Action(loraISR);

  // Start listening
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] Listening...");
  } else {
    Serial.printf("[LoRa] startReceive failed, code: %d\n", state);
  }
}

void handleLoRaReceive() {
  if (!loraReceived) return;
  loraReceived = false;

  uint8_t buffer[256];
  int len = radio.getPacketLength();

  int state = radio.readData(buffer, len);

  if (state == RADIOLIB_ERR_NONE && len >= sizeof(MeshMessage) - 200) {
    config.lastRssi = radio.getRSSI();
    config.lastSnr = radio.getSNR();

    MeshMessage* msg = (MeshMessage*)buffer;

    Serial.printf("[LoRa] RX from %s, type=%d, hops=%d, RSSI=%.0f, SNR=%.1f\n",
                  msg->srcId, msg->type, msg->hops, config.lastRssi, config.lastSnr);

    // Check if we've seen this message
    if (isMessageSeen(msg->msgId, msg->srcId)) {
      Serial.println("[LoRa] Duplicate message, ignoring");
      radio.startReceive();
      return;
    }
    markMessageSeen(msg->msgId, msg->srcId);

    // Process based on type
    switch (msg->type) {
      case MSG_DISCOVERY: {
        // Extract name from data
        char name[32] = {0};
        if (msg->dataLen > 0) {
          strncpy(name, msg->data, min((int)msg->dataLen, 31));
        }
        updateDeviceList(msg->srcId, name, config.lastRssi);

        // Respond with our info
        char response[64];
        snprintf(response, sizeof(response), "%s", config.deviceName);
        sendLoRaMessage(MSG_ACK, msg->srcId, response, strlen(response));
        break;
      }

      case MSG_ACK: {
        char name[32] = {0};
        if (msg->dataLen > 0) {
          strncpy(name, msg->data, min((int)msg->dataLen, 31));
        }
        updateDeviceList(msg->srcId, name, config.lastRssi);
        break;
      }

      case MSG_HEARTBEAT: {
        updateDeviceList(msg->srcId, "", config.lastRssi);
        break;
      }

      case MSG_DATA:
      case MSG_BROADCAST: {
        // Check if message is for us or broadcast
        bool forUs = (strcmp(msg->dstId, config.shortId) == 0) ||
                     (strcmp(msg->dstId, "*") == 0) ||
                     (msg->dstId[0] == '*');

        if (forUs) {
          // Forward to BLE client
          char bleMsg[300];
          snprintf(bleMsg, sizeof(bleMsg),
                   "{\"type\":\"lora\",\"from\":\"%s\",\"data\":\"%.*s\",\"rssi\":%d}",
                   msg->srcId, msg->dataLen, msg->data, config.lastRssi);
          sendBleMessage(bleMsg);
          Serial.printf("[LoRa] Message for us: %.*s\n", msg->dataLen, msg->data);
        }

        // Relay if broadcast and we're parent, or if direct and not for us
        if (config.role == ROLE_PARENT && msg->hops < MESH_MAX_HOPS) {
          if (strcmp(msg->dstId, "*") == 0 || !forUs) {
            relayMessage(msg);
          }
        }
        break;
      }

      default:
        Serial.printf("[LoRa] Unknown message type: %d\n", msg->type);
    }
  } else if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] Read error: %d\n", state);
  }

  // Restart receive
  radio.startReceive();
}

void sendLoRaMessage(MeshMsgType type, const char* destId, const char* data, uint8_t dataLen) {
  if (loraTransmitting) {
    Serial.println("[LoRa] Already transmitting, queuing...");
    return;
  }

  MeshMessage msg;
  msg.type = type;
  msg.hops = 0;
  msg.msgId = ++msgIdCounter;
  strncpy(msg.srcId, config.shortId, 6);
  strncpy(msg.dstId, destId ? destId : "*", 6);
  msg.dataLen = min(dataLen, (uint8_t)200);
  if (data && dataLen > 0) {
    memcpy(msg.data, data, msg.dataLen);
  }

  loraTransmitting = true;

  size_t msgSize = sizeof(MeshMessage) - 200 + msg.dataLen;
  int state = radio.transmit((uint8_t*)&msg, msgSize);

  loraTransmitting = false;

  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] TX to %s, type=%d, len=%d\n", msg.dstId, type, msgSize);
  } else {
    Serial.printf("[LoRa] TX failed, code: %d\n", state);
  }

  // Back to receive mode
  radio.startReceive();
}

void relayMessage(MeshMessage* msg) {
  msg->hops++;
  Serial.printf("[LoRa] Relaying message, hops=%d\n", msg->hops);

  size_t msgSize = sizeof(MeshMessage) - 200 + msg->dataLen;
  int state = radio.transmit((uint8_t*)msg, msgSize);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] Relay failed, code: %d\n", state);
  }

  radio.startReceive();
}

void broadcastDiscovery() {
  Serial.println("[LoRa] Broadcasting discovery...");
  sendLoRaMessage(MSG_DISCOVERY, "*", config.deviceName, strlen(config.deviceName));
}

void sendHeartbeat() {
  sendLoRaMessage(MSG_HEARTBEAT, "*", nullptr, 0);
}

#else
// Wokwi stubs
void initLoRa() {
  Serial.println("[LoRa] Skipped in Wokwi simulation");
}
void handleLoRaReceive() {}
void sendLoRaMessage(MeshMsgType type, const char* destId, const char* data, uint8_t dataLen) {
  Serial.printf("[LoRa-SIM] Would send: type=%d, dest=%s, data=%.*s\n",
                type, destId ? destId : "*", dataLen, data);
}
void broadcastDiscovery() { Serial.println("[LoRa-SIM] Discovery broadcast"); }
void sendHeartbeat() { Serial.println("[LoRa-SIM] Heartbeat"); }
#endif

// ================= MESSAGE TRACKING =================
bool isMessageSeen(uint16_t msgId, const char* srcId) {
  unsigned long now = millis();
  for (int i = 0; i < MESH_MAX_SEEN; i++) {
    if (seenMessages[i].msgId == msgId &&
        strcmp(seenMessages[i].srcId, srcId) == 0 &&
        (now - seenMessages[i].timestamp) < MESH_MSG_TTL) {
      return true;
    }
  }
  return false;
}

void markMessageSeen(uint16_t msgId, const char* srcId) {
  seenMessages[seenIndex].msgId = msgId;
  strncpy(seenMessages[seenIndex].srcId, srcId, 6);
  seenMessages[seenIndex].srcId[6] = '\0';
  seenMessages[seenIndex].timestamp = millis();
  seenIndex = (seenIndex + 1) % MESH_MAX_SEEN;
}

void updateDeviceList(const char* id, const char* name, int8_t rssi) {
  // Check if device exists
  for (int i = 0; i < deviceCount; i++) {
    if (strcmp(devices[i].id, id) == 0) {
      devices[i].rssi = rssi;
      devices[i].lastSeen = millis();
      if (name && strlen(name) > 0) {
        strncpy(devices[i].name, name, 31);
      }
      return;
    }
  }

  // Add new device
  if (deviceCount < MAX_DEVICES) {
    strncpy(devices[deviceCount].id, id, 6);
    devices[deviceCount].id[6] = '\0';
    strncpy(devices[deviceCount].name, name ? name : "Unknown", 31);
    devices[deviceCount].rssi = rssi;
    devices[deviceCount].lastSeen = millis();
    deviceCount++;
    Serial.printf("[MESH] New device: %s (%s)\n", id, name);

    // Notify BLE client
    char bleMsg[128];
    snprintf(bleMsg, sizeof(bleMsg),
             "{\"type\":\"device\",\"id\":\"%s\",\"name\":\"%s\",\"rssi\":%d}",
             id, name, rssi);
    sendBleMessage(bleMsg);
  }
}

// ================= BLE FUNCTIONS =================
void sendBleMessage(const char* message) {
  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue((uint8_t*)message, strlen(message));
    pTxCharacteristic->notify();
    Serial.printf("[BLE] Sent: %s\n", message);
  }
}

void updateDeviceInfo() {
  if (pDeviceInfoCharacteristic) {
    char info[192];
    snprintf(info, sizeof(info),
             "{\"id\":\"%s\",\"short\":\"%s\",\"name\":\"%s\",\"role\":%d,\"ble\":%s}",
             config.deviceId, config.shortId, config.deviceName, config.role,
             config.bleConnected ? "true" : "false");
    pDeviceInfoCharacteristic->setValue((uint8_t*)info, strlen(info));
  }
}

void handleBleCommand(const char* message) {
  if (strstr(message, "\"cmd\":\"ping\"")) {
    char response[128];
    snprintf(response, sizeof(response),
             "{\"type\":\"pong\",\"id\":\"%s\",\"short\":\"%s\"}",
             config.deviceId, config.shortId);
    sendBleMessage(response);
  }
  else if (strstr(message, "\"cmd\":\"setName\"")) {
    char* nameStart = strstr(message, "\"data\":\"");
    if (nameStart) {
      nameStart += 8;
      char* nameEnd = strchr(nameStart, '"');
      if (nameEnd) {
        size_t len = min((size_t)(nameEnd - nameStart), sizeof(config.deviceName) - 1);
        strncpy(config.deviceName, nameStart, len);
        config.deviceName[len] = '\0';
        updateDeviceInfo();
        sendBleMessage("{\"type\":\"ack\",\"cmd\":\"setName\"}");
      }
    }
  }
  else if (strstr(message, "\"cmd\":\"setRole\"")) {
    char* dataStart = strstr(message, "\"data\":");
    if (dataStart) {
      int role = atoi(dataStart + 7);
      config.role = (DeviceRole)role;
      updateDeviceInfo();
      sendBleMessage("{\"type\":\"ack\",\"cmd\":\"setRole\"}");
    }
  }
  else if (strstr(message, "\"cmd\":\"getInfo\"")) {
    char response[256];
    snprintf(response, sizeof(response),
             "{\"type\":\"info\",\"id\":\"%s\",\"short\":\"%s\",\"name\":\"%s\",\"role\":%d}",
             config.deviceId, config.shortId, config.deviceName, config.role);
    sendBleMessage(response);
  }
  else if (strstr(message, "\"cmd\":\"getDevices\"")) {
    // Send list of discovered devices
    char response[512];
    int offset = snprintf(response, sizeof(response), "{\"type\":\"devices\",\"list\":[");
    for (int i = 0; i < deviceCount; i++) {
      if (i > 0) offset += snprintf(response + offset, sizeof(response) - offset, ",");
      offset += snprintf(response + offset, sizeof(response) - offset,
                         "{\"id\":\"%s\",\"name\":\"%s\",\"rssi\":%d}",
                         devices[i].id, devices[i].name, devices[i].rssi);
    }
    snprintf(response + offset, sizeof(response) - offset, "]}");
    sendBleMessage(response);
  }
  else if (strstr(message, "\"cmd\":\"discover\"")) {
    broadcastDiscovery();
    sendBleMessage("{\"type\":\"ack\",\"cmd\":\"discover\"}");
  }
  else if (strstr(message, "\"cmd\":\"sendLora\"")) {
    // Extract destination and message
    char* destStart = strstr(message, "\"dest\":\"");
    char* dataStart = strstr(message, "\"data\":\"");
    char dest[7] = "*";
    char data[200] = {0};

    if (destStart) {
      destStart += 8;
      char* destEnd = strchr(destStart, '"');
      if (destEnd) {
        size_t len = min((size_t)(destEnd - destStart), sizeof(dest) - 1);
        strncpy(dest, destStart, len);
        dest[len] = '\0';
      }
    }

    if (dataStart) {
      dataStart += 8;
      char* dataEnd = strchr(dataStart, '"');
      if (dataEnd) {
        size_t len = min((size_t)(dataEnd - dataStart), sizeof(data) - 1);
        strncpy(data, dataStart, len);
        data[len] = '\0';
      }
    }

    if (strlen(data) > 0) {
      sendLoRaMessage(MSG_DATA, dest, data, strlen(data));
      sendBleMessage("{\"type\":\"ack\",\"cmd\":\"sendLora\"}");
    }
  }
  else {
    Serial.printf("[BLE] Unknown command: %s\n", message);
  }
}

void initBLE() {
  Serial.println("[BLE] Initializing...");

  char bleName[32];
  snprintf(bleName, sizeof(bleName), "LoRaMesh-%s", config.shortId);

  NimBLEDevice::init(bleName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHAR_TX_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  pRxCharacteristic = pService->createCharacteristic(
    CHAR_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pDeviceInfoCharacteristic = pService->createCharacteristic(
    CHAR_DEVICE_INFO_UUID,
    NIMBLE_PROPERTY::READ
  );

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  NimBLEDevice::startAdvertising();

  Serial.printf("[BLE] Advertising as: %s\n", bleName);
}

// ================= MAIN SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  ESP32-S3 LoRa Mesh - Full Version");
  Serial.println("========================================\n");

  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_BLE, OUTPUT);
  pinMode(BTN_MODE, INPUT_PULLUP);

  blinkLED(LED_STATUS, 3, 100);

  generateDeviceId();
  snprintf(config.deviceName, sizeof(config.deviceName), "Device-%s", config.shortId);
  config.role = ROLE_STANDALONE;
  config.bleConnected = false;
  config.lastRssi = 0;
  config.lastSnr = 0;

  Serial.printf("[SYS] Device ID: %s\n", config.deviceId);
  Serial.printf("[SYS] Short ID: %s\n", config.shortId);
  Serial.printf("[SYS] Name: %s\n", config.deviceName);

  initBLE();
  initLoRa();
  updateDeviceInfo();

  // Initial discovery broadcast
  delay(random(100, 1000));  // Random delay to avoid collisions
  broadcastDiscovery();

  digitalWrite(LED_STATUS, HIGH);
  Serial.println("\n[SYS] Ready!\n");
}

// ================= MAIN LOOP =================
unsigned long lastHeartbeat = 0;
unsigned long lastDiscovery = 0;
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;

void loop() {
  // Handle LoRa receive
  #ifndef WOKWI_SIMULATION
  handleLoRaReceive();
  #endif

  // Handle BLE messages
  if (newBleMessage) {
    newBleMessage = false;
    handleBleCommand(rxBuffer);
  }

  // Button handling
  bool buttonPressed = (digitalRead(BTN_MODE) == LOW);
  if (buttonPressed && !buttonWasPressed) {
    buttonPressStart = millis();
    buttonWasPressed = true;
  }
  else if (!buttonPressed && buttonWasPressed) {
    unsigned long pressDuration = millis() - buttonPressStart;
    buttonWasPressed = false;

    if (pressDuration > 3000) {
      config.role = ROLE_STANDALONE;
      Serial.println("[BTN] Reset to standalone");
      blinkLED(LED_STATUS, 5, 100);
    }
    else if (pressDuration > 50) {
      config.role = (DeviceRole)((config.role + 1) % 3);
      Serial.printf("[BTN] Role: %d\n", config.role);
      blinkLED(LED_STATUS, config.role + 1, 200);
    }
    updateDeviceInfo();
  }

  // Periodic heartbeat (every 30s)
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }

  // Periodic discovery (every 2 min, only if parent)
  if (config.role == ROLE_PARENT && millis() - lastDiscovery > 120000) {
    lastDiscovery = millis();
    broadcastDiscovery();
  }

  // Clean up old devices (not seen in 5 min)
  for (int i = 0; i < deviceCount; i++) {
    if (millis() - devices[i].lastSeen > 300000) {
      // Remove device by shifting array
      for (int j = i; j < deviceCount - 1; j++) {
        devices[j] = devices[j + 1];
      }
      deviceCount--;
      i--;
    }
  }

  // Status LED
  if (millis() - lastHeartbeat < 1000) {
    // Recently sent heartbeat, quick flash
  } else if (!deviceConnected) {
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 2000) {
      lastBlink = millis();
      digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    }
  }

  delay(10);
}
