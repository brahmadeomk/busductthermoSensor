# Testing - Busduct thermo sensor (Arduino Nano, Modbus RTU)

Two levels of testing:

1. **Host tests** - the sketch and the Modbus library compiled and run on a PC
   with simulated serial port, clock, ADC, Timer1 and EEPROM. Done, all pass.
2. **Hardware bench tests** - on a real Nano with RS485 and a Modbus master.
   Procedure and record sheet below. **Not yet done.**

---

## 1. Host tests (PC)

### How to run

Linux or WSL with `g++`:

```sh
tests/host/run_host_tests.sh
```

The script syntax-checks both source files against stub Arduino headers, then
builds and runs two test programs with AddressSanitizer and UBSan (catches
out-of-bounds access and undefined behaviour). The `tests/` folder is outside
the sketch folder, so the Arduino IDE never compiles it.

| File | Purpose |
|---|---|
| `tests/host/stubs/` | Minimal Arduino / AVR stand-ins (`Arduino.h`, `EEPROM.h`, `avr/wdt.h`) |
| `tests/host/test_modbus.cpp` | Modbus library with simulated UART and `micros()` |
| `tests/host/test_sensor.cpp` | Real Timer2 ISR + `processTemperature()` + EEPROM code with simulated ADC values and Timer1 |

### Results (last run: all 43 pass)

**Modbus library (20)**

| # | Case | Result |
|---|---|---|
| 1 | FC3 read of all registers, CRC and byte order | PASS |
| 2 | Frame for another slave ID ignored, no reply | PASS |
| 3 | Bad CRC ignored, no reply | PASS |
| 4 | FC3 address out of range -> exception 2 | PASS |
| 5 | FC3 count that wraps the address -> exception 3 | PASS |
| 6 | FC3 count 0 -> exception 3 | PASS |
| 7 | FC6 write to a measurement register -> exception 2, value unchanged | PASS |
| 8 | FC6 write to slave ID register, exact echo reply | PASS |
| 9 | Broadcast (ID 0) write ignored | PASS |
| 10 | FC16 write to slave ID register | PASS |
| 11 | Unsupported function (FC4) -> exception 1 | PASS |
| 12 | Two frames merged without silence -> no bogus reply | PASS |
| 13 | 2.5 ms gap inside a frame still assembled (USB converter) | PASS |
| 14 | 70-byte frame overflow -> no reply, no memory error | PASS |
| 15 | Normal request answered after an overflow | PASS |
| 16 | Registers appended after the writable one are readable | PASS |
| 17 | Slave ID (21) writable with registers appended after it | PASS |
| 18 | Write to the last register (not writable) -> exception 2 | PASS |
| 19 | 32 registers read in one request (69-byte reply) | PASS |
| 20 | Interface counters: bus frames, requests, CRC, frame errors, exceptions | PASS |

**Sensor sketch (23)**

| # | Case | Register value | Result |
|---|---|---|---|
| 21 | 25 degC (ADC 511) | 2498 | PASS |
| 22 | 25 degC with dither 510/511 (sub-LSB averaging) | 2500 | PASS |
| 23 | 100 degC (ADC 66/67) | 9998 | PASS |
| 24 | 0 degC (ADC 788) | 4 | PASS |
| 25 | NTC open (ADC 1023) | -32768 | PASS |
| 26 | Status bit 0 set on open | 1 | PASS |
| 27 | NTC short (ADC 0) | 32767 | PASS |
| 28 | Status bit 1 set on short | 2 | PASS |
| 29 | Intermittent open (2 % of samples at 1023) -> fault | -32768 | PASS |
| 30 | 0.5 % spikes (below fault threshold) -> still a temperature | 2475 | PASS |
| 31 | Temperature recovers after a fault | 2498 | PASS |
| 32 | Status bits cleared after recovery | 0 | PASS |
| 33 | ADC average register (0.1 count) | 5110 | PASS |
| 34 | Uptime counts 1 per 1 s window | 9 | PASS |
| 35 | Boot count = 1 on erased EEPROM | 1 | PASS |
| 36 | Slave ID change saved, no save-fail bit | - | PASS |
| 37 | Boot count = 2 after second start | 2 | PASS |
| 38 | Slave ID retained after restart | 40 | PASS |
| 39 | Slave ID unchanged at end | 40 | PASS |
| 40 | ISR time, 80 cycles -> 5.0 us | 50 | PASS |
| 41 | ISR time follows longer runs -> 10.0 us | 100 | PASS |
| 42 | ISR time follows shorter runs -> 2.5 us | 25 | PASS |
| 43 | ISR max since power-up keeps highest | 100 | PASS |

Case 30 shows that rare spikes bias the average slightly (-0.25 degC at 0.5 %
spikes to full scale). Spike rejection can be added if the bench test shows it
is needed.

### What host tests cannot show

Real AVR timing (`micros()`, UART, ADC conversion time), the RS485 transceiver,
EEPROM behaviour on power loss, electrical noise, and the AVR compiler itself
(the code has not been compiled with avr-gcc yet). These are covered by the
bench tests below.

---

## 2. Hardware bench tests

### Equipment

- Arduino Nano with the sensor board (or Nano + MAX485 module, 100.5 kohm divider resistor, 100 nF on A0 and AREF)
- USB-RS485 converter and a PC Modbus master: QModMaster, Modbus Poll or similar
- 120 ohm termination at both bus ends
- Resistors to simulate the NTC (1 % or better), or a resistance decade box
- The real NTC and a reference thermometer
- Optional: a second slave (or second ID polled on the bus), bench supply with adjustable voltage

Master settings: **9600 baud, 8N1, slave ID 71** (default on erased EEPROM),
function 3, holding registers, start 0, count 34 (all registers). Temperature
is register 3, signed 16-bit, 0.01 degC.

### NTC simulation resistors

Put the resistor in place of the NTC (A0 to GND). Expected values from the
beta model (R25 = 100 kohm, beta 3950, fixed resistor 100.5 kohm):

| Simulated temp | Resistor | Expected reg 3 | Expected reg 23 (ADC x10) |
|---|---|---|---|
| 0 degC | 336.2 kohm | 0 | 7883 |
| 25 degC | 100.0 kohm | 2500 | 5107 |
| 50 degC | 35.88 kohm | 5000 | 2694 |
| 60 degC | 24.86 kohm | 6000 | 2031 |
| 80 degC | 12.70 kohm | 8000 | 1149 |
| 100 degC | 6.975 kohm | 10000 | 665 |
| 120 degC | 4.071 kohm | 12000 | 399 |

Use the nearest standard value and calculate the expected temperature from the
measured resistance. Tolerance: a 1 % resistor shifts the reading by about
0.25 degC at 25 degC and 0.3 degC at 100 degC. **Pass: within +/-0.5 degC of the
value expected for the measured resistance.**

### Test procedure

Record actual values and Pass/Fail in the sheet at the end.

| ID | Test | Steps | Expected / pass criteria |
|---|---|---|---|
| H1 | Build and upload | Arduino IDE, board "Arduino Nano", correct processor (bootloader). Compile, note flash/RAM, upload. | No errors or warnings that matter; RAM use well below 2048 bytes |
| H2 | First power-up | Erase EEPROM first (or use a new board). Power up, read 0-33. | Reg 21 = 71, reg 31 = 1, regs 29/30 counting up 1 per s, reserved regs (0-2, 4-19) = 0 |
| H3 | Modbus read | Poll 0-33 every 1 s for 5 min. | No timeouts, no CRC errors on the master; reg 25 rises with each poll; reg 26, 27, 28 stay 0 |
| H4 | Temperature accuracy | Fit each resistor from the table, wait 3 s, read reg 3 and 23. | Within +/-0.5 degC of expected; reg 23 matches table within about +/-30 (0.1 count) |
| H5 | Update rate | Watch reg 3 while changing the resistor; watch reg 30. | Reg 3 settles within 2 s; reg 30 increases by 1 each second |
| H6 | Noise / stability | Fixed 100 kohm, log reg 3 for 60 s. | Max - min <= 5 (0.05 degC) |
| H7 | Real NTC | Fit the real NTC next to a reference thermometer at room temp and in warm water/oven (40-80 degC). | Within NTC tolerance (typically +/-1 degC) |
| H8 | NTC open | Disconnect the NTC. | Within 2 s: reg 3 = -32768, reg 22 bit 0 = 1 |
| H9 | NTC short | Short A0 to GND. | Within 2 s: reg 3 = 32767, reg 22 bit 1 = 1 |
| H10 | Recovery | Reconnect the NTC / remove short. | Normal temperature within 2 s, reg 22 = 0 |
| H11 | Loose contact | Wiggle a loose NTC wire for 10 s. | Fault values may appear while wiggling; no reset (reg 31 unchanged), no Modbus timeouts |
| H12 | Slave ID change | FC6: write 72 to reg 21 at ID 71. Wait 100 ms. Read reg 21 at ID 72. | Ack from ID 71; ID 72 answers with reg 21 = 72; ID 71 no longer answers |
| H13 | ID retained | Power-cycle 3 times. Read at ID 72. | ID 72 answers every time; reg 31 increased by 3 |
| H14 | Invalid ID | Write 0, then 248, to reg 21. | Reg 21 stays at current ID; reg 22 bit 2 = 0 |
| H15 | Write protection | Write to reg 3; FC16 write to reg 21 with 2 registers; broadcast (ID 0) write to reg 21. | Exception 2 for the first two; broadcast changes nothing |
| H16 | Power loss during ID change | Write a new ID and cut power within 50 ms; repeat 10 times with alternating IDs. | After each power-up the device answers on either the old or the new ID, never 71 or anything else |
| H17 | Multi-drop bus | Master polls this ID and a non-existent ID (e.g. 99) alternately, 100 ms scan, 1 h. | No timeouts for this ID; reg 24 counts all frames; reg 26/27 stay 0; reg 20 < 3000 |
| H18 | ISR timing | Read reg 32 and 33 during H17. | Reg 32 about 30-60 (3-6 us); reg 33 < 150 (15 us) |
| H19 | Loop timing | Read reg 20 during H17 and during an ID change (H12). | Normally a few hundred us; < 3000 except the second of the ID change (EEPROM write ~30 ms) |
| H20 | Exception counter | Send FC4 or read address 40. | Exception reply; reg 28 increases |
| H21 | Supply variation | If a bench supply is available: 4.5 V to 5.5 V at 100 kohm. | Reg 3 changes < 0.1 degC (ratiometric) |
| H22 | Legacy migration | Flash the old firmware, set its ID to 80, then flash this firmware. | Comes up on ID 80 (not 71) |
| H23 | Long run | 24 h continuous polling, real NTC. | No timeouts; reg 31 unchanged (no resets); reg 26/27 not rising; reg 33 stable |

### If a test fails

| Symptom | Check |
|---|---|
| No answer at all | Baud/parity/ID, A/B swapped, DE and /RE both on pin 3, termination |
| CRC errors rising (reg 26) | Wiring, missing termination or bias resistors, ground between converter and board |
| Frame errors rising (reg 27) | Reg 20 above 3000 us (loop too slow) or master sending oversized frames |
| Temperature offset at all points | R_FIXED value, NTC R0/beta in the sketch, AREF and divider not on the same 5 V node |
| Temperature noisy | 100 nF on A0 missing, AREF not decoupled, cable routing next to power conductors |
| Reg 31 rising without power cycles | Brown-outs: supply dips, BOD fuse level |
| ID lost after power cycle | Reg 22 bit 2, EEPROM wear, BOD fuse (EEPROM writes at low voltage) |

---

## 3. Record sheet

Board / serial no.: ____________  Firmware commit: ____________  Date: ____________  Tester: ____________

| ID | Measured / observed | Pass / Fail | Notes |
|---|---|---|---|
| H1 | Flash: ____ bytes, RAM: ____ bytes | | |
| H2 | | | |
| H3 | | | |
| H4 | 0: ____  25: ____  50: ____  60: ____  80: ____  100: ____  120: ____ | | |
| H5 | | | |
| H6 | max - min: ____ | | |
| H7 | ref: ____ / sensor: ____ ; ref: ____ / sensor: ____ | | |
| H8 | | | |
| H9 | | | |
| H10 | | | |
| H11 | | | |
| H12 | | | |
| H13 | | | |
| H14 | | | |
| H15 | | | |
| H16 | | | |
| H17 | | | |
| H18 | reg 32: ____  reg 33: ____ | | |
| H19 | reg 20: ____ | | |
| H20 | | | |
| H21 | | | |
| H22 | | | |
| H23 | | | |
