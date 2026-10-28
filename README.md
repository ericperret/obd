<!--
  Name     : README.md
  Purpose  : overview of the obd_can_bridge project (matches firmware v8.4)
  Author   : Eric PERRET (FY4AY / ex-F1OCM)
  Date     : 2026-10-27
  Version  : 8.4.1
  License  : CC BY-NC 4.0 — commercial use forbidden without the author's written permission (eperret@azs.fr)
-->

# obd_can_bridge

🇫🇷 [Version française : LISEZMOI.md](LISEZMOI.md)

A **standalone OBD-II diagnostic bridge on ESP32-S3**, with a built-in web interface reachable over WiFi from a smartphone or a PC.

The firmware is a **single `.ino` file**, HTML/CSS/JavaScript page included (a string literal embedded in the program). **No external CAN library**: the MCP2515 is driven through its SPI registers.

Developed and validated on:

- **Citroën Nemo 1.3 HDi**
- Fiat **199A2000 / 1.3 Multijet** engine (PSA F13DTE5)
- **Marelli MJD 8F3.F6** engine control unit
- engine C-CAN bus, **500 kbit/s**
- **ISO 15765-4** diagnostics, **29-bit extended addressing**

**Current firmware: v8.4**

## Hardware

| Item | Reference |
|---|---|
| MCU | Seeed XIAO ESP32-S3 |
| CAN | Seeed CAN Bus Breakout 713-105100001 |
| CAN controller | MCP2515, 16 MHz crystal |
| Transceiver | SN65HVD230 |
| CS | D7 |
| INT | D6 (wired, unused — the firmware polls) |
| Bus | engine CAN, 500 kbit/s |

## WiFi interface

The XIAO ESP32-S3 creates its own access point:

- **SSID: `NEMO-OBD`**
- **Password: `nemo1234`**
- **IP: `192.168.1.1`**
- interface: `http://192.168.1.1/`

No Internet connection required, no external resource (no CDN, no JS dependency).

## What the standard imposes on this vehicle

ISO 15765-4 restricts diagnostic addressing to two PDU Format values:

- `0x18DA<TA><SA>` — physical addressing (one given ECU)
- `0x18DB33<SA>` — functional addressing (broadcast, all OBD ECUs)

**Empirical finding on this vehicle**: the BSI gateway only routes **functional** requests to the engine bus. Unicast physical addressing (`0x18DA10F1`) never gets an answer, while broadcast (`0x18DB33F1`) always works. The firmware therefore sends **all** its Mode 01 requests as broadcast, and filters the answers on the known ECU address.

**Responding ECU identified**: `0x10` (engine, Marelli MJD 8F3.F6). It is the only control unit of the vehicle that answers legal generic OBD — ABS/ESP, airbag and BSI sit on a separate bus, not wired in this setup.

**Mode 01 PIDs actually supported by this ECU**: the index chain stops at `0x40` (bitmap `80 C0 00 00`, last bit at 0) — no PID beyond `0x4A` is available in generic OBD on this vehicle (no engine torque, no direct boost pressure, no engaged gear position).

## Hardware filtering

The MCP2515 is set with a hardware mask/filter that only accepts `0x18DAF1xx` frames (physical OBD answer, whatever the responding ECU) — all the rest of the bus traffic (which quickly saturates the 2 receive buffers once the engine runs) is dropped by the silicon before it even reaches SPI.

## Web page features

### Continuous log (gauges)

- 9 circular gauges (SVG, green ring on black): **Pedal D, Pedal E, Speed, RPM, Load, MAF, Intake pressure, Rail pressure, EGR**.
- **LOG** / **STOP LOG** button.
- Normal rate: about **1 Hz** (one full cycle of the 9 PIDs, aligned on the second).
- Automatic detection of power loss — switch to **BURST** (15 s, cycles chained with a short 100 ms pause, no 1 Hz alignment) when:
  - the pedal rises **and** the speed drops compared with the previous cycle, **or**
  - the pedal is above 75 %.
- CAN **read** order (fixed, for synchronisation): `0x49→0x0D→0x0C→0x04→0x10→0x0B→0x23→0x2C→0x4A` — pedal and speed read back to back at the start of the cycle, minimal time gap between the two values that trigger the detection.
- Gauge **display** order independent of the read order (logical visual grouping).
- Time-stamped log (`t_ms` relative to start-up) written to `/drivelog.csv` on LittleFS.
- Flash writes **buffered in RAM**, at most once every **5 s**, **never during a burst** (a flash write can freeze WiFi while it runs — the whole buffer is written at once on return to normal mode).
- LittleFS fill bar (red above 85 %).
- **PURGER LOG** button (deletes the log without downloading it; refused while a log is running).
- Separate download (`/download_log`), file `nemo_drivelog.csv`.

### Monitor status (level 2, no button)

Shown below the gauges, updated automatically, no interaction needed:

- **OBD standard** (`0x1C`) — queried **only once at start-up** (static value, never changes for a given vehicle).
- **Monitors since DTC clear** (`0x01`) and **current drive cycle monitors** (`0x41`) — bit-by-bit decoding (MIL, number of codes, ignition type, ratio of ready/complete monitors over supported monitors), queried **every 60 s**, never during a burst.

### PID scan (discovery of supported PIDs)

- **CALL** button.
- Follows the standard index chain `0x00 → 0x20 → 0x40 → …` (least significant bit of each bitmap = "the next group exists"), stops as soon as a bitmap says no or a link of the chain does not answer.
- Only queries the PIDs actually announced as supported by the decoded bitmaps (no blind 0x00–0xFF sweep).
- Live results (Server-Sent Events): PID, YES/NO answer, MCP2515 buffer (RXB0/RXB1), raw data, decoded value.
- Results written to `/result.csv`, downloadable (`/download`).

### Single PID test

- **TEST PID 0x0C** button — probes one fixed PID (`TEST_PID`, editable in the code), to check the link quickly without a full scan.

## Mode 01 decoding

A lookup table (~155 entries) covers the Mode 01 PIDs with a simple linear formula (`value = mul × raw + add`, on 1, 2 or 4 bytes, signed or not, at a given offset in the frame — it also handles multi-value PIDs such as `0x66` or `0x9A`). Formulas sourced from SAE J1979 / Wikipedia.

**Not decoded (shown as "Brut", raw)**: the remaining bit-field PIDs not covered by level 2 above — `0x00`, `0x03`, `0x12`, `0x13`, `0x1D`, `0x1E`, `0x51`, `0x5F` and similar. `0x01`, `0x1C` and `0x41` are decoded separately (see "Monitor status" above).

**Documented anomaly, not a bug**: PID `0x01` (byte B, bit 3 — ignition type) answers "petrol" on this diesel vehicle. The bit is read as the standard says (checked by triangulating sources, consistent with the real captured data); the Marelli ECU answers that way, it is not a decoding error.

## Reliability — problems met and fixes

| Observed symptom | Actual cause | Fix |
|---|---|---|
| ~45 % of PIDs without answer | Next request sent too soon after a fast answer (minimum rest time violated) | Fixed **80 ms** cycle per PID (≥ N_Bs = 75 ms, ISO 15765-4 table 6), fast answer or not |
| STOP LOG and page refresh blocked during logging | `server.handleClient()` called only once per PID (~80–720 ms apart) | Called on every iteration of the CAN wait loop |
| Total freeze, even when disconnected (no vehicle) | Without an acknowledgement (ACK) from another CAN node, the MCP2515 retransmits forever internally; the TXREQ bit never clears, blocking every further send | Wait bounded to 10 ms, then forced abort of the previous transmission |
| Progressive freeze on a long drive | Flash write (`flush()`) on every cycle, which can freeze WiFi while it runs | RAM buffer, spaced flash writes (5 s), never during a burst |

Debug instrumentation (`Serial.print`, 115200 baud) remains in the code — cheap, and already useful once to locate the freeze above.

## Build

```bash
 ./obd_build.sh flash
 ./obd_build.sh mon

or

arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 obd_can_bridge.ino
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32S3 -p /dev/ttyACM0 obd_can_bridge.ino
```

No third-party library to install.

## Fault codes (DTC)

- **CHECK DEFAUTS** (check faults): reads Mode 03 (stored), 07 (pending), 0A (permanent) as broadcast. Accepted limit: 3 codes max per mode (single-frame ISO-TP only, no multi-frame — beyond that, untestable in real use anyway).
- **CLEAR DEFAUTS** (clear faults): Mode 04, mandatory JS confirmation (reminder of the effect on readiness monitors). Refused by the ECU (NRC `22`) while the engine runs — standard behaviour, confirmed on this vehicle.
- Each decoded code (`P`/`C`/`B`/`U` + 4 digits) comes with its description in French, from a table of **986 generic codes** embedded in the firmware (source: EOBD-Facile translation PDF, checked and corrected: one content conflict settled by visual check of the document, several layout offsets corrected, 3 codes left out because undefined in the source).

## To do

- Nothing identified for now on the DTC / fault features.

## License

**Free for non-commercial use** — [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/), with mandatory credit of the source and the author:

> obd — Eric PERRET — https://github.com/ericperret/obd

Any commercial use is forbidden without the author's written permission, requested by e-mail: **eperret@azs.fr**.
Full terms (French / English): [LICENSE](LICENSE).

## Author

**Eric Perret (FY4AY / ex-F1OCM)**
Project developed and maintained to diagnose the Citroën Nemo 1.3 HDi.
