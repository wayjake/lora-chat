# LoRa Chat - ESP32-S3 Mesh Messaging

A mesh networking system using ESP32-S3 + SX1262 LoRa modules with Web Bluetooth control. Send messages between devices up to 2.8km apart without WiFi or cellular.

## Features

- **LoRa Mesh Networking**: Multi-hop message relay up to 5 hops
- **Web Bluetooth Interface**: Control from Chrome on laptop/phone
- **Device Discovery**: Automatic detection of nearby LoRa devices
- **Parent/Child Architecture**: Hierarchical device management
- **Long Range**: Up to 2.8km line-of-sight communication

## Hardware

Designed for ESP32-S3 + SX1262 boards like [this one](https://www.amazon.com/dp/B0D2WXLZXQ):

| Spec | Value |
|------|-------|
| MCU | ESP32-S3FN8 (240MHz dual-core, 8MB Flash) |
| LoRa | SX1262 (863-928MHz) |
| Range | Up to 2.8km (open air) |
| Bluetooth | BLE 5.0 + Classic |
| Battery | 3000mAh 3.7V LiPo support |
| Sensitivity | -139dBm (SF12, 125kHz) |

## Project Structure

```
lora-chat/
├── src/
│   └── main.cpp          # ESP32 firmware (BLE + LoRa)
├── web/
│   └── index.html        # Web Bluetooth control app
├── platformio.ini        # PlatformIO config
├── wokwi.toml           # Wokwi simulation config
├── diagram.json         # Wokwi circuit diagram
└── README.md
```

## Quick Start

### 1. Install PlatformIO

Install [PlatformIO](https://platformio.org/) extension in VS Code or Cursor.

### 2. Configure Pin Mapping

Edit `platformio.ini` to match your board's SX1262 pins:

```ini
build_flags =
    -DLORA_CS=8
    -DLORA_DIO1=14
    -DLORA_RST=12
    -DLORA_BUSY=13
    -DLORA_FREQ=915.0  ; Use 868.0 for EU
```

### 3. Build & Flash

```bash
# Build
pio run -e esp32-s3

# Upload
pio run -e esp32-s3 -t upload

# Monitor serial output
pio device monitor
```

### 4. Run Web App

```bash
cd web
python -m http.server 8080
```

Open http://localhost:8080 in Chrome.

## Wokwi Simulation

For testing without hardware (BLE only, no LoRa):

1. Install **Wokwi for VS Code** extension
2. Build: `pio run -e wokwi`
3. Open `diagram.json` → Start Simulation

## Mesh Protocol

### Message Types

| Type | Code | Description |
|------|------|-------------|
| BROADCAST | 0x01 | Send to all devices |
| DIRECT | 0x02 | Send to specific device |
| DISCOVERY | 0x03 | Find nearby devices |
| ACK | 0x04 | Acknowledgment |
| HEARTBEAT | 0x05 | Periodic keepalive |
| DATA | 0x10 | User message |

### Message Structure

```c
struct MeshMessage {
  uint8_t  type;       // Message type
  uint8_t  hops;       // Hop count (max 5)
  uint16_t msgId;      // Unique message ID
  char     srcId[6];   // Source device ID
  char     dstId[6];   // Destination ("*" = broadcast)
  uint8_t  dataLen;    // Payload length
  char     data[200];  // Payload
};
```

## BLE Commands

Send JSON commands from the web app:

```json
{"cmd":"ping"}                              // Ping device
{"cmd":"getInfo"}                           // Get device info
{"cmd":"setName","data":"MyDevice"}         // Set device name
{"cmd":"setRole","data":1}                  // Set role (0/1/2)
{"cmd":"discover"}                          // Broadcast discovery
{"cmd":"getDevices"}                        // List discovered devices
{"cmd":"sendLora","dest":"*","data":"Hi"}   // Send LoRa message
```

## Device Roles

| Role | Code | Behavior |
|------|------|----------|
| Standalone | 0 | Independent, no relay |
| Parent | 1 | Primary hub, relays messages, periodic discovery |
| Child | 2 | Reports to parent |

## Hardware Controls

**Button (GPIO 15)**:
- Short press: Cycle roles (Standalone → Parent → Child)
- Long press (3s): Reset to Standalone

**LEDs**:
- GPIO 2 (Status): Slow blink = idle, solid = connected
- GPIO 4 (BLE): On when BLE client connected

## LoRa Configuration

Default settings optimized for range:

| Parameter | Value |
|-----------|-------|
| Frequency | 915 MHz (US) / 868 MHz (EU) |
| Bandwidth | 125 kHz |
| Spreading Factor | 9 |
| Coding Rate | 4/7 |
| TX Power | 14 dBm |

## Troubleshooting

**LoRa not initializing?**
- Check pin definitions in `platformio.ini`
- Verify SPI connections to SX1262
- Check serial output for error codes

**Web Bluetooth not connecting?**
- Use Chrome or Edge (not Firefox/Safari)
- Must be HTTPS or localhost
- Try `chrome://flags/#enable-web-bluetooth`

**Devices not discovering each other?**
- Ensure same frequency band
- Check for antenna connection
- Reduce distance for initial testing

## Future Improvements

- [ ] Message encryption (AES-128)
- [ ] Persistent config (NVS storage)
- [ ] GPS location sharing
- [ ] Message acknowledgment/retry
- [ ] Power saving modes
- [ ] OTA firmware updates

## License

MIT License - Feel free to use and modify.
