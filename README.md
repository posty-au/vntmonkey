# Hungry Monkey v2.9

### Hardware-Timed Deterministic Injector Sequencer

Hungry Monkey v2.9 is a high-precision engine management architecture designed for extreme performance applications. Unlike traditional software-only ECUs that rely on jitter-prone CPU interrupts, the Hungry Monkey utilizes a dual-layer strategy: a high-level **Cortex-M7** for complex "thinking" and a discrete logic layer for deterministic "doing".

## 🏎️ Key Features

* 
**Hardware-Timed Precision**: Employs a dedicated **Si5351A clock generator** to provide a 1 MHz timebase, ensuring a fixed **1 µs resolution** for injector pulses.


* 
**Zero-Jitter Execution**: Offloads time-critical pulse execution to **IS62WV25616 Parallel SRAM** and discrete counters, bypassing MCU overhead and software latency.


* 
**GaN Power Stage**: Features **LMG3411R150 GaN drivers** for ultra-fast switching, high thermal efficiency, and robust solenoid control.


* 
**Hardened Safety**: A dedicated **LOOP_ENABLE master kill switch** provides a physical hardware interlock, instantly disabling injection if safety limits (EGT, Boost, RPM) are exceeded.


* 
**Wide & Fast Telemetry**: Native Ethernet support for high-bandwidth **UDP telemetry** (up to 10ms intervals) and real-time map updates.



## 🛠️ Hardware Architecture

The system consists of several beautifully interlocking discrete components:

1. 
**Si5351A Clock**: Master timing source producing 1 MHz for hardware counters and 8 kHz for deterministic control loops.


2. 
**Parallel SRAM**: Stores 12 injector entries, allowing instantaneous, hardware-direct access to pulse data.


3. 
**74HC161 Counters**: The system's "time axis," incrementing every 1 µs to schedule injection events.


4. 
**CD74HC688 Comparator**: The "edge detector" that identifies the exact moment "time == pulse width" to end injection.


5. 
**74HC08 AND Gates**: Final logic layer for per-cylinder enabling, phase grouping, and hardware safety gating.



## 💻 Firmware Capabilities

The Hungry Monkey v2.9 firmware is optimized for the **Teensy 4.1** and includes:

* 
**16x16 Tuning Tables**: Full bilinear interpolation for VNT, LPG, and Wastegate control.


* 
**Safety Hysteresis**: Hardcoded limits (e.g., **EGT > 900°C**, **Boost > 2.2 bar**) with intelligent recovery thresholds.


* 
**Config Persistence**: Dynamic loading/saving of system parameters (IP, turbo settings, trims) from an SD card.


* 
**Hardware Config**: DIP switch support to change cylinder counts (3–12) or phases without reflashing code.



## 🚀 Getting Started

1. **Hardware**: Flash the latest to a Teensy 4.1.
2. 
**SD Setup**: Place `config.txt`, `vnt_map.csv`, `lpg_map.csv`, and `wg_map.csv` on a FAT32 formatted SD card.


3. **Networking**: Connect via Ethernet. The default IP is `192.168.1.177`.


4. 
**UDP Interface**: Use the dedicated command port (8888) for tuning and the telemetry port (8889) for real-time data.



## 📜 License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.

---

*Designed and built by **posty-au***.
