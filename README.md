# Rollus - 433.92 MHz RF Roller Shutter Control & Remote Emulation for ESPHome

An ESPHome component and firmware configuration for bidirectional control of proprietary 433.92 MHz RF roller shutter motors using an **ESP32** and a **CC1101** transceiver.

This project enables complete local control via Home Assistant: it emulates original remote controls, receives and decodes signals from physical remotes to keep shutter states synchronized, persists rolling code counters in non-volatile storage (NVS), and provides diagnostic programming buttons for pairing new motors.

---

## Features

- **Bidirectional RF Communication (433.92 MHz):**
  - **Transmitter:** Emulates physical remotes with exact Manchester encoding and checksum calculation to control shutters directly.
  - **Receiver & State Sync:** Listens for physical remote presses, decodes incoming telegrams, updates Home Assistant cover states in real time, and synchronizes rolling counters.
- **Universal, Data-Driven Protocol Engine (`rollus_protocol.h`):**
  - Zero hardcoded remote IDs or series conditionals in the core protocol engine.
  - Every remote is defined by its individual masks directly in `rollus.yaml`.
- **Automatic Remote Sniffer & Log Assistant:**
  - Pressing an unregistered physical remote automatically analyzes the signal, computes the exact parity and checksum masks, and prints a ready-to-copy 1-line configuration directly into the ESPHome log.
- **Home Assistant Cover & Diagnostic Integration:**
  - Time-based cover entities with configurable open/close travel durations (`number` config entities).
  - Dedicated **Diagnostic Programming Buttons** (`entity_category: diagnostic`) to trigger the pairing command (`0xA1`, UP+DOWN) directly from Home Assistant.
- **NVS Counter Persistence:**
  - Rolling code counters for every remote are saved to ESP32 Flash (NVS) to ensure continuity across reboots and power outages.
- **Interactive Code Calculator (`rolladen_rechner.html`):**
  - Offline web tool to simulate, verify, and calculate rolling codes, checksums, and hex frames for any remote ID.

---

## Hardware Setup

| ESP32 Pin | CC1101 Pin | Function |
| :--- | :--- | :--- |
| **3.3V** | VCC | Power Supply (3.3V only) |
| **GND** | GND | Ground |
| **GPIO 18** | SCK | SPI Clock |
| **GPIO 19** | MISO / SO | SPI Master In Slave Out |
| **GPIO 23** | MOSI / SI | SPI Master Out Slave In |
| **GPIO 5** | CSN / CS | SPI Chip Select |
| **GPIO 21** | GDO0 | Transmitter Output (remote_transmitter) |
| **GPIO 22** | GDO2 | Receiver Input (remote_receiver) |

---

## Protocol Overview

- **Frequency & Modulation:** 433.92 MHz, ASK/OOK.
- **Encoding:** 64-bit Manchester encoding with ~360 µs half-bit duration.
- **Telegram Structure (8 Bytes):**

```text
[ B0 ] [ B1 ] [ B2 ] [ B3 ] [ B4 ] [ B5 ] [ B6 ] [ B7 ]
  │      │      │      │      │      │      │      └── Checksum via linear polynomial matrix V
  │      │      │      │      │      │      └───────── B7 ^ mask_b6_b7
  │      │      │      │      │      └──────────────── Hardware / dialect identifier (e.g. 0xD8 or 0x6E)
  │      │      │      │      └─────────────────────── Remote ID (0x00 - 0xFF)
  │      │      │      └────────────────────────────── Parity: B2 ^ B4 ^ b3_parity
  │      │      └───────────────────────────────────── Command byte (0x81=UP, 0x41=STOP, 0x21=DOWN, 0xA1=PROG)
  │      └──────────────────────────────────────────── Protocol Header (always 0x01)
  └─────────────────────────────────────────────────── Rolling code carrier: B6 ^ c
```

- **Commands:**
  - `0x81`: Open / UP
  - `0x41`: Stop
  - `0x21`: Close / DOWN
  - `0xA1`: Program / Pairing (UP + DOWN simultaneously)

---

## Quickstart & Configuration

### 1. Register Shutters in `rollus.yaml`

In `rollus.yaml`, add your shutters under `on_boot` using the compact 1-line registration syntax:

```yaml
# Syntax: rollus.register_shutter(ID, id(cover_entity), {b5, mask_b6_b7, b3_parity, b7_up, b7_stop, b7_down, jump_mode});
on_boot:
  - priority: 600
    then:
      - lambda: |-
          rollus.set_transmitter_fn([](const std::vector<int32_t> &pulses) {
            auto call = id(rf_transmitter).transmit();
            call.get_data()->set_data(pulses);
            call.perform();
          });

          // Examples:
          rollus.register_shutter(0x2D, id(rolladen_1), {0xD8, 0xC4, 0x1D, 0x7A, 0x25, 0x89, JUMP_DYNAMIC});
          rollus.register_shutter(0x39, id(rolladen_3), {0x6E, 0xE6, 0x89, 0x01, 0x5E, 0xF2, JUMP_XOR});
          rollus.register_shutter(0xB9, id(rolladen_4), {0x6E, 0xE6, 0x89, 0x30, 0x6F, 0xC3, JUMP_XOR});
```

### 2. Discovering a New / Unknown Remote

1. Flash the ESP32 and open the ESPHome log: `esphome logs rollus.yaml`.
2. Press any button on the physical remote control.
3. The sniffer will automatically analyze the transmission and output the exact configuration line:

```text
[W][Rollus]: Unbekannte FB -> ID: 0x39 | Taste: HOCH | T: 227
[W][Rollus]: >>> In rollus.yaml unter on_boot einfuegen:
[W][Rollus]:     rollus.register_shutter(0x39, id(rolladen_X), {0x6E, 0xE6, 0x89, 0x01, 0x5E, 0xF2, JUMP_XOR});
```

4. Copy the line into your `rollus.yaml`, define your `time_based` cover entity, and you are ready to go!

### 3. Diagnostic Programming Buttons (Pairing)

To pair a motor with a new ID or put the motor into programming mode without touching the physical remote, add diagnostic button entities in `rollus.yaml`:

```yaml
button:
  - platform: template
    name: "Schlafzimmer klein Programmieren"
    id: btn_prog_3
    entity_category: diagnostic
    icon: "mdi:cog-sync"
    on_press:
      - lambda: rollus.send(0x39, 0xA1);
```

These buttons are categorized under **Diagnostic** in Home Assistant, keeping your main dashboard clean while giving you one-click motor pairing access whenever needed.

---

## Acknowledgments & Credits

Special thanks to GitHub user **[@GuentherP](https://github.com/GuentherP)** for his foundational reverse-engineering work and in-depth analysis of the 433.92 MHz RF signal structure. 

His insights and discussion in [SignalDuinoCpp Discussion #2](https://github.com/GuentherP/SignalDuinoCpp/discussions/2#discussioncomment-18776578) laid the crucial groundwork for decoding the frame layout, Manchester timing, and checksum logic.

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).
