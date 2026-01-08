# LoRa Chat - Project Status

> Last Updated: 2026-01-08

## Current Phase: Hardware Setup

---

## Quick Status

| Area | Status |
|------|--------|
| Firmware | Ready for testing |
| Web App | Ready |
| Hardware | Awaiting delivery |
| Testing | Not started |

---

## Next Steps

### 1. Hardware Setup
- [ ] Receive ESP32-S3 + SX1262 board
- [ ] Identify correct SX1262 pin mapping from board schematic
- [ ] Update `platformio.ini` with correct pins:
  ```ini
  -DLORA_CS=?
  -DLORA_DIO1=?
  -DLORA_RST=?
  -DLORA_BUSY=?
  ```
- [ ] Connect USB and verify board is detected

### 2. Initial Flash & Test
- [ ] Build firmware: `pio run -e esp32-s3`
- [ ] Flash to board: `pio run -e esp32-s3 -t upload`
- [ ] Open serial monitor: `pio device monitor`
- [ ] Verify startup messages (BLE advertising, LoRa init)
- [ ] Note any LoRa init errors and adjust pins if needed

### 3. BLE Testing
- [ ] Start web server: `cd web && python -m http.server 8080`
- [ ] Open http://localhost:8080 in Chrome
- [ ] Click "Connect to Device"
- [ ] Verify device appears as "LoRaMesh-XXXXXX"
- [ ] Test ping command
- [ ] Test role switching

### 4. LoRa Testing (requires 2+ devices)
- [ ] Flash second ESP32 board
- [ ] Set one device as Parent, one as Child
- [ ] Verify devices discover each other
- [ ] Send test message between devices
- [ ] Verify message appears in web app
- [ ] Test range (start close, increase distance)

### 5. Field Testing
- [ ] Test outdoors for range measurement
- [ ] Document actual range achieved
- [ ] Test multi-hop relay (3+ devices)
- [ ] Test battery operation

---

## Backlog / Future Features

### High Priority
- [ ] Message encryption (AES-128)
- [ ] Persistent config storage (NVS)
- [ ] Message retry/acknowledgment system
- [ ] Improve web app UI for mobile

### Medium Priority
- [ ] GPS location sharing
- [ ] Power saving / deep sleep modes
- [ ] OTA firmware updates
- [ ] Message history storage

### Low Priority
- [ ] Custom PCB design
- [ ] Enclosure design
- [ ] Multi-channel support
- [ ] Mesh routing optimization

---

## Hardware Notes

**Board**: ESP32-S3 + SX1262 (Amazon B0D2WXLZXQ)

| Spec | Value |
|------|-------|
| MCU | ESP32-S3FN8 |
| Flash | 8MB |
| LoRa | SX1262 (863-928MHz) |
| Range | ~2.8km (claimed) |
| Battery | 3000mAh 3.7V |

**Pin Mapping** (to be confirmed):
```
LORA_CS   = ?  (update after checking schematic)
LORA_DIO1 = ?
LORA_RST  = ?
LORA_BUSY = ?
```

---

## Issues & Blockers

<!-- Add any issues encountered here -->

| Issue | Status | Notes |
|-------|--------|-------|
| - | - | No issues yet |

---

## Testing Log

<!-- Record test results here -->

| Date | Test | Result | Notes |
|------|------|--------|-------|
| - | - | - | No tests yet |

---

## Commands Reference

```bash
# Build
pio run -e esp32-s3

# Flash
pio run -e esp32-s3 -t upload

# Monitor
pio device monitor

# Web app
cd web && python -m http.server 8080

# Wokwi simulation
pio run -e wokwi
# Then open diagram.json in VS Code/Cursor
```

---

## Links

- **Repo**: https://github.com/wayjake/lora-chat
- **Hardware**: https://www.amazon.com/dp/B0D2WXLZXQ
- **RadioLib Docs**: https://jgromes.github.io/RadioLib/
- **Web Bluetooth API**: https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API
