# Wideband Autonomous Module (WAM) v1.0

**Target MCU:** Teensy 4.1 (i.MX RT1062)

**Design intent:** Hardware-lock-safe wideband controller supporting LSU-class sensors, GaN heater drive, isolated analog front-end, UDP telemetry, SD profile storage, and 0–3 V analog lambda output.

---

## 1. Core Processing & Timing

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| U1 | 1 | **Teensy 4.1** | Main MCU, native Ethernet PHY, SD slot used |
| Y1 | 1 | Onboard | Teensy-integrated oscillator |

---

## 2. Power Input & Protection

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| F1 | 1 | Automotive blade or resettable fuse (3–5 A) | Heater supply protection |
| D1 | 1 | **SMBJ36A** TVS diode | Load-dump & surge suppression |
| L1 | 1 | Ferrite bead (≥3 A) | EMI isolation for logic rail |
| C1 | 1 | 100 µF, 35 V electrolytic | Bulk input capacitance |
| C2–C3 | 2 | 1 µF ceramic, X7R | High-frequency decoupling |

---

## 3. Heater Power Stage (GaN)

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| Q1 | 1 | **GaN FET, ≥40 V** (e.g. EPC2218 / GS61008) | Low-side PWM heater switch |
| U2 | 1 | GaN gate driver (e.g. LM5113 / isolated if needed) | Ensures fast, clean switching |
| Rg | 1 | 5–10 Ω resistor | Gate damping |
| D2 | 1 | Fast diode or TVS | Heater inductive transient clamp |
| Rsense (opt) | 1 | 0.01–0.05 Ω, 1% | Optional heater current telemetry |

**Notes:**
- Heater ground isolated from analog ground, star-connected at supply entry.
- PWM ≥100 Hz per firmware contract.

---

## 4. Pump Cell Analog Front-End (ADC A0)

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| U3 | 1 | **OPA320** / **TLV9062** | RRIO, low-noise op-amp (TIA) |
| Rf | 1 | 100–200 Ω, 0.1% | Pump current to voltage conversion |
| Cf | 1 | 100–470 pF C0G | TIA stability |
| Rfilt | 1 | 100 Ω | ADC source impedance control |
| Cfilt | 1 | 1 nF C0G | Anti-aliasing filter |

**Contract:** Source impedance <1 kΩ @ ADC.

---

## 5. Nernst Cell Sense (ADC A1)

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| U4 | 1 | Same op-amp as U3 | Buffer / sense amplifier |
| Rbias | 1 | Precision resistor network | Nernst excitation |
| TP1 | 1 | Test point | Calibration / future firmware expansion |

**Note:** Hardware supports impedance-based health monitoring.

---

## 6. DAC – Analog Lambda Output (0–3 V)

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| U5 | 1 | **MCP4725** (I2C) | 12-bit DAC |
| U6 | 1 | Op-amp buffer (optional) | Required if ECU ADC <10 kΩ |
| Rout | 1 | 100 Ω | Output isolation |
| Cout | 1 | 1 µF | Output smoothing |

---

## 7. RTC & Timebase

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| U7 | 1 | **DS3231** | I2C RTC, temperature-compensated |
| B1 | 1 | CR2032 | RTC backup |

---

## 8. Ethernet Interface

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| J1 | 1 | RJ45 MagJack | Per PJRC reference design |
| Ceth | 4 | 0.1 µF | PHY decoupling |
| FBeth | 2 | Ferrite beads | Ethernet noise isolation |

---

## 9. Indicators & IO

| Ref | Qty | Part | Notes |
|---|---:|---|---|
| LED1 | 1 | Green LED | READY indicator |
| LED2 | 1 | Red LED | FAULT indicator |
| Rled | 2 | 330–1 kΩ | LED current limit |
| J2 | 1 | Sensor connector | LSU-class pinout |
| J3 | 1 | Programming / debug header | Optional |

---

## 10. Passive Infrastructure

| Item | Notes |
|---|---|
| Decoupling capacitors | 0.1 µF at every IC power pin |
| Analog reference cap | Placed at Teensy ADC ref pin |
| Ground stitching vias | Separate analog / power planes |

---

## 11. Firmware–Hardware Lock Notes

- ADC settling assumes <1 kΩ source impedance
- Heater noise must not contaminate analog ground
- Ethernet and GaN switching isolated via ferrites
- All calibration data stored on SD (`sensor_profile.txt`)

---

**Status:** Preliminary BoM — suitable for schematic capture and first PCB spin (v1.0 silicon).

