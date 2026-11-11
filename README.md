# Texecom Premier 412 Smart Alarm (ESP8266 + Telegram + Wintex over Wi-Fi)

**[Türkçe](README.tr.md)**

Turn a classic, offline **Texecom Premier 412** burglar alarm into a smart one with a **€5 NodeMCU (ESP8266)**. You get live status, notifications, remote arming and PIN-protected disarming in Telegram. There is no cloud service, no server and no monthly fee. The panel keeps working exactly as before.

The ESP8266 talks to the panel's **Com1** port using the Wintex/UDL protocol, the same one Texecom's Wintex software uses. It also bridges Wintex over Wi-Fi, so you can program the panel from your PC without opening it.

> ⚠️ This is a hobby project and is not affiliated with Texecom. It talks to a security device: test it carefully, keep your alarm's normal operation in mind, and use it at your own risk.

## Features

**Telegram bot (runs on the ESP8266 itself)**
- One live dashboard message with buttons, updated in place. It refreshes itself for 3 minutes after opening.
- Armed, disarmed, exit and entry time, ready status, active zones and battery/system voltage.
- Notifications: armed or disarmed and **by whom**, entry time started, siren/alarm, motion while armed, link lost or restored, and low voltage.
- **Remote arming** with confirmation and a 30-second exit countdown (⚡ arm now / ❌ cancel). It only arms when every zone is quiet.
- **Remote disarming** with an on-screen PIN keypad. Wrong PINs are announced to everyone, and 3 wrong PINs lock disarming for 15 minutes.
- Pages for zones (last activity, today's count), event history, daily statistics with bar charts, and yesterday's summary.
- Per-notification settings stored in EEPROM, a nightly "you forgot to arm" reminder, and a 09:00 daily summary.
- **Family access:** up to 5 accounts. New members tap **"📱 Share my number"**. Telegram verifies that the number belongs to that account, and the bot accepts it only if it is on your list.
- Every other Telegram account gets no information at all.
- TLS with certificate validation (Go Daddy root pinned in `telegram_root.h`).

**Panel link**
- Its own UDL session: login, ~4 reads per second, heartbeat. It only sends `Z`, `R`, `H`, plus `A`/`D` when you arm or disarm from Telegram.
- **Wintex over Wi-Fi:** "Connect Via Network" to port `10001`. The ESP pauses its own session while Wintex is connected and resumes afterwards.
- A debug monitor on port `23` with a protocol decoder, raw hex tools and a built-in **logic analyser** (`scope`). The analyser found the two bugs described below.

## Hardware

| Part | Notes |
|---|---|
| NodeMCU V3 (ESP8266, CH340) | Any ESP8266 board works; pins below are NodeMCU labels |
| MP1584EN buck converter | Com1 12 V → ~4.9 V for VIN, trimmed before connecting |
| 4-channel BSS138 level shifter | Only one channel is used (panel TX 5 V → ESP 3.3 V) |
| 4 female–female jumpers | From Com1 to the box |
| 3D-printed box (optional) | [`enclosure/`](enclosure/) (OpenSCAD + STL) |

### Wiring

Com1 (JP10) is a 5-position Molex header with four pins. Seen from the front, left to right, the pins are 5, 4, 3, (gap), 1:

| Com1 pin | Signal | Goes to |
|---|---|---|
| 1 | +12 V | MP1584 IN+ |
| 3 | GND | MP1584 IN− |
| 4 | Panel RX (idles ~4.8 V via the panel's own pull-up) | NodeMCU **D5** (GPIO14), **direct** |
| 5 | Panel TX (5 V) | Level shifter HV1 → LV1 → NodeMCU **D7** (GPIO13) |

Inside the box:

- MP1584 OUT+ → NodeMCU VIN and level shifter HV.
- MP1584 OUT− → NodeMCU G and both level shifter GND pins.
- NodeMCU 3V3 → level shifter LV.

Leave D4, D8, RX and TX unconnected.

- **Don't cut the panel's power** to wire it: on many installs that sounds the external siren. Connect live, GND first.
- **Don't power from USB and VIN at the same time.**
- GPIO3 (the board's RX pin) can't be used: the on-board CH340 holds it high.

## Two things that cost two days (read this first)

1. **The line is 19200 8N2, in both directions.** With 1 stop bit the panel answers every session request with `03 0F ED` (hang up). This is easy to misread as a "clean reply".
2. **Disable the ESP's internal pull-up on the RX pin.** The core's UART swap (`pinMode(13, FUNCTION_4)`) and EspSoftwareSerial both enable it. With it on, the start of every reply was corrupted and only the tail arrived. The firmware now uses EspSoftwareSerial on D7 with `enableRxGPIOPullUp(false)`. With that change, the measured success rate went from 0/20 to 19/20 valid replies.

The `scope` command toggles TX itself with interrupts disabled and samples D7. It is how both bugs were found.

## Setup

1. **Panel (via Wintex once):** set Com1 to PC-Com and note the UDL passcode. For remote control, tick Partition 1 → *Enable Remote Arming* / *Enable Remote Disarming*.
2. **Secrets:** copy [`texecom_sniffer/secrets.example.h`](texecom_sniffer/secrets.example.h) to `secrets.h` and fill in the following. `secrets.h` is git-ignored.
   - Wi-Fi, OTA password
   - UDL as raw digits: `1234` → `{1,2,3,4}`
   - Telegram bot token from @BotFather, and your chat id
   - Allowed phone numbers and names
   - Disarm PIN, which should be different from your alarm code
3. **Build** with arduino-cli (esp8266 core 3.1.x, library EspSoftwareSerial, bundled):
   ```bash
   arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --output-dir build texecom_sniffer
   arduino-cli upload -p COM3 --fqbn esp8266:esp8266:nodemcuv2 --input-dir build texecom_sniffer
   # later, over Wi-Fi:
   arduino-cli upload -p <ESP_IP> -l network --fqbn esp8266:esp8266:nodemcuv2 -F password=<OTA_PASS> --input-dir build texecom_sniffer
   ```
4. **Zone names:** edit `ZONE_NAMES` in the `.ino`.
5. **Wintex over Wi-Fi:** Panel Details → *Connect Via Network*, host = ESP IP, port `10001`.

## Protocol

Frames are `[len][cmd][data…][checksum]`. The checksum is the sum of the previous bytes XOR 0xFF.

| What | Command |
|---|---|
| Login | bare `Z` → serial no., then `Z` + UDL digits (raw bytes) → product string ("Premier 412 VF.04") |
| Zones (8 × 1 byte, `01` = active) | `R 0x0004F0 08` |
| Partition block | `R 0x000D6B 13`: +7 entry `01` / exit `10`, +8 armed, +9 ready, +6 siren/strobe, +0x12 arm failed |
| Voltages | `R 0x000DCE 04`: raw × 18/256 V, from one reading |
| Arm / disarm area 1 | `A 01` / `D 01`, ACK `06`. Remote arming has **no exit time**, so the bot adds its own countdown. |

Full notes (Turkish): [`docs/PROTOCOL.tr.md`](docs/PROTOCOL.tr.md). Earlier work this builds on:

- [shuckc/pialarm](https://github.com/shuckc/pialarm): Wintex captures for the Premier Elite.
- [RoganDawes' ESPHome Wintex component](https://github.com/RoganDawes/esphome/tree/wintex_v2/esphome/components/wintex): Premier 832.

## Security

- **Ports 23 (monitor) and 10001 (Wintex bridge) have no authentication.** Anyone on your Wi-Fi can reach them. Never port-forward them. Keep the ESP on a trusted network or VLAN.
- Disarming over UDL needs no user code, so the Telegram PIN and lockout are the real protection. Enable Telegram two-step verification on every family account.
- The bot ignores every Telegram account that isn't on the list.

## Limitations

These are known gaps and things not yet verified:

- Tested on one Premier 412 (V15.04) only. Addresses may differ on the 816/832/Elite.
- Burglar, panic, fire and tamper bits have not been verified with a real alarm (only the siren byte has). Mains-fail detection isn't decoded yet.
- History and counters live in RAM and reset on reboot. If Wi-Fi or the ESP dies, the bot can't tell you. An external heartbeat is planned.
- Keypad arming shows as "keypad", because the user isn't read from the panel log yet.

## License

[MIT](LICENSE)
