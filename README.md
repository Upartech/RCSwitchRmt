# RCSwitchRmt

[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform: ESP32](https://img.shields.io/badge/platform-ESP32-lightgrey.svg)](#)
[![Upartech](https://img.shields.io/badge/author-Upartech-orange.svg)](#)

**RCSwitchRmt** is an ESP32-focused, multitasking RF OOK library built on top of the **RMT** peripheral.  
It is **inspired by** the classic [`rc-switch`](https://github.com/sui77/rc-switch) project but redesigned for **modern ESP32 devices**, offering a fully non-blocking and hardware-driven architecture for RF communication.

RCSwitchRmt is an advanced ESP32 library that leverages the RMT peripheral integrated in most ESP32 microcontrollers to handle precise, hardware-timed 433/315 MHz OOK (On-Off Keying) communication asynchronously and without blocking the main thread. Inspired by the classic RCSwitch project, it provides full multitasking transmit (TX) and receive (RX) support for ASK-type RF modules such as WL102, RX470, RX500, RX18210, RX531, WL101, FS1000A, MX-RM-5V, T1L, R1L, R1A, T3MAX, R1A-M3, R1A-M5N, STX882, SRX882, STX883Pro, SRX883Pro, T2L_433, H34P, T3-433, H3V4F, H3V3E, E160-T4MS1, E160-R4MS1, TYJM01, RXB8, SYN480R, RFM119, RFM119S, SF R315A, R433A, R433D, 34N01, STX885, among others.
It is compatible with a wide range of common protocols used in remote controls and wireless switches, including PT2262, PT2260, EV1527, SC5211, HS2240, HT6P20B, HT12E, RS-200, and more.
By combining the precision of RMT with FreeRTOS multitasking, RCSwitchRmt achieves high reliability, non-blocking performance, and full compatibility with standard OOK/ASK wireless communication systems.

> ⚠️ **Note:** This library is designed exclusively for **ESP32 family boards with the RMT peripheral** (ESP-IDF v5+ or Arduino-ESP32 ≥ 3.x).  
> It will not work on AVR, ESP8266, or any MCU without RMT support.

---

## Why RCSwitchRmt?

- **Non-blocking TX/RX:** Uses FreeRTOS tasks and the ESP32’s RMT peripheral for true asynchronous operation — your main loop and other services remain fully responsive.
- **Hardware-level precision:** The RMT peripheral handles digital pulse generation and decoding in hardware, ideal for OOK, Manchester, or any pulse-width-based signaling.
- **Independent TX/RX protocols:** Transmit and receive can use completely different protocol definitions.
- **Custom protocol builder:** Define your own RF protocol with **2 or 4 timing segments (HLHLHLHL)** per symbol for flexible encoding design.
- **Configurable protocol behavior:** Both TX and RX can have their protocol, tolerance, separation limit, and expected bit length adjusted independently.
- **Automatic protocol recognition:** RX can automatically identify the most probable protocol (like rc-switch), though reception is **faster and more stable** when the protocol is defined explicitly.
- **Multitasking design:** Dedicated FreeRTOS tasks manage transmission and reception while RMT performs precise timing at hardware level.
- **Protocol compatibility:** Fully supports all rc-switch protocols — works with the same common 433 / 315 MHz OOK remote outlets and transmitters.
- **Efficient, reliable, and flexible:** Hardware-timed, non-blocking, multitask operation maximizes throughput while minimizing CPU usage.

---

## Features

- Transmit (TX) and receive (RX) 433 / 315 MHz OOK signals using the ESP32 **RMT** peripheral.  
- Asynchronous, multitask design (no need to call `repeater()` manually).  
- **Independent configuration for TX and RX protocols.**  
- **Custom user-defined protocols** with 2 or 4 timing segments (HLHLHLHL).  
- Send behavior configurable by:
  - **Repetition count (`setRepeatTransmit(n)`):** number of frames per `send()` call.  
  - **Minimum transmit time (`setMinTransmitTime(ms)`):** continuous transmission window.  
  - Priority is given to **repetition count (n > 1)** over time-based mode.  
  - Setting milliseconds = 0 disables time-based sending (single burst only).  
- RX: auto or manual protocol selection, tolerance and separation tuning.  
- Debug logging available through `RMT_OOK_DEBUG`.  
- Compatible with rc-switch device protocols: SC5262 / SC5272, HX2262 / HX2272, PT2262 / PT2272, EV1527 / RT1527 / FP1527 / HS1527, Intertechno outlets HT6P20X
- **Dual RX modes:**
  - *Instant mode* — access the latest received frame directly.  
  - *Buffered mode* — stores the last 32 frames (default), configurable buffer depth, ideal for processing after heavy Wi-Fi, ESP-NOW, or MQTT operations.  
  The buffer can be read sequentially using: bool read(Decoded* out);

  
---

## How `send()` works

The `send(code, bitlen)` function automatically adapts its behavior based on your configuration:

- If you call `setRepeatTransmit(n)` with **n > 1**, each `send()` call transmits *n* frames (burst-count mode).  
- If you call `setMinTransmitTime(ms)` with **ms > 0**, `send()` keeps transmitting continuously during that time window (time-based mode).  
- If **both are set**, **repetition count has priority** — time-based mode is ignored.  
- Setting `ms = 0` disables time-based transmission completely (default).  
- Setting `n = 1` returns `send()` to single-burst behavior.

This dual behavior makes `send()` flexible and suitable for different use cases — from rapid control bursts to timed continuous transmission.

---

## Requirements

- Any ESP32 variant with **RMT peripheral** (ESP32, S2, S3, C3, C6, etc.).  
- ESP-IDF ≥ 5.0 or Arduino-ESP32 ≥ 3.0 (core based on IDF 5).  
- PlatformIO target: `espressif32`.

---

## Installation

**Arduino IDE**

1. Download or clone this repository into your `Documents/Arduino/libraries/` folder.  
2. Restart the Arduino IDE.  
3. Once published, it can be installed directly via the **Arduino Library Manager**.

**PlatformIO**

Add to your project’s `platformio.ini`:

```ini
lib_deps = 
  https://github.com/Upartech/RCSwitchRmt.git
