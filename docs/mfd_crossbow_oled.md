# MFD Crossbow Backpack with OLED Status Display

This adds two build targets to the MFD Crossbow Antenna Tracker backpack so the
backpack can show live GPS telemetry, bind status, and WiFi-update info on a
128×64 SSD1306 OLED.

| Target | Hardware |
| --- | --- |
| `MFD_Crossbow_HeltecWifiKit32_Backpack` | Heltec WiFi Kit 32 V2 (built-in OLED) |
| `MFD_Crossbow_ESP32_OLED_Backpack` | Any generic ESP32 dev board + external SSD1306 0.96" I2C module |

Both targets run the same firmware. They differ only in build-time pin macros
and a few Heltec-specific quirks (Vext power gate, dim-panel brightness
overrides, display inversion).

## What you see on the OLED

- **Running** — GPS coordinates, sats, fix type, altitude, ground speed,
  heading, and a `LINK` / `----` indicator driven by the freshness of the last
  CRSF telemetry packet.
- **Binding** — instruction screen prompting you to send a bind packet from
  the TX with the matching binding phrase.
- **WiFi update** — SSID (`ExpressLRS VRx Backpack`), password (`expresslrs`),
  and the OTA URL (`http://10.0.0.1`).

The display refreshes at ~4 Hz to keep the I2C bus from contending with the
10 Hz MAVLink stream to the tracker.

## Heltec WiFi Kit 32 V2

### Pin map

| Function | GPIO |
| --- | --- |
| OLED SDA | 4 |
| OLED SCL | 15 |
| OLED reset | 16 |
| Vext gate (active LOW) | 21 |
| Onboard LED | 25 |
| Boot / PRG button | 0 |
| MAVLink TX to tracker (`Serial`) | 1 |

### Build and flash

```
pio run -e MFD_Crossbow_HeltecWifiKit32_Backpack_via_UART -t upload
```

On Windows, with PIO installed but not on PATH:

```
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e MFD_Crossbow_HeltecWifiKit32_Backpack_via_UART -t upload
```

The Heltec V2 panel is dim by reputation. The firmware sets the SSD1306
charge-pump, pre-charge, VCOMH and contrast registers to their maximum values
and enables display inversion (lit background, dark text) to maximize
perceived brightness.

## Generic ESP32 + external SSD1306

### Hardware

- Any ESP32 dev board (ESP32-DevKitC, NodeMCU-32S, DOIT ESP32, etc.)
- A standalone 128×64 SSD1306 I2C OLED module (4-pin: VCC, GND, SDA, SCL)

### Wiring

| OLED module | ESP32 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Connect MAVLink output to the Crossbow tracker:

| ESP32 | Tracker |
| --- | --- |
| GPIO 1 (TX0) | GPS-input RX |
| GND | GND |

### Build and flash

```
pio run -e MFD_Crossbow_ESP32_OLED_Backpack_via_UART -t upload
```

No Vext, no display inversion, no reset pin — most standalone SSD1306 modules
have their own RC reset circuit.

## Binding phrase and WiFi credentials

Set them in `user_defines.txt` at the repo root:

```
-DMY_BINDING_PHRASE="your-phrase-here"
-DHOME_WIFI_SSID="YourWiFi"
-DHOME_WIFI_PASSWORD="YourPassword"
```

`user_defines.txt` is local — do not commit it.

## OTA flashing after the first UART flash

Power-cycle the device three times in quick succession to reboot into WiFi
update mode. The OLED will then show the SSID and URL. Then build with the
`_via_WIFI` env:

```
pio run -e MFD_Crossbow_HeltecWifiKit32_Backpack_via_WIFI -t upload
# or for the generic ESP32 build:
pio run -e MFD_Crossbow_ESP32_OLED_Backpack_via_WIFI -t upload
```

PlatformIO uploads the new firmware to `http://elrs_vrx.local/update` via the
backpack's HTTP update server.

## Build-flag reference

These are the macros the OLED code keys off — useful if you want to add a new
board variant.

| Macro | Required | Meaning |
| --- | --- | --- |
| `OLED_SSD1306` | yes | Enables OLED code paths and pulls in U8g2. |
| `OLED_SDA`, `OLED_SCL` | yes | I2C pins for the panel. |
| `OLED_RST` | optional | GPIO wired to the panel's reset pin. Omit if the module self-resets. |
| `HELTEC_VEXT` | optional | GPIO that gates power to the OLED (active LOW). Heltec-only. |
| `OLED_INVERT` | optional | Render with lit background. Useful for dim panels. |
