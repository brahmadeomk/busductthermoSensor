# busductthermoSensor

Arduino Nano (ATmega328P, 16 MHz) Modbus RTU slave for busduct monitoring:
NTC temperature on A0, ratiometric with external AREF (5 V).

## Files

```
busductThermoSensor/
  busductThermoSensor.ino   sketch: sampling, register update, slave ID in EEPROM
  SimpleModbusSlave.h/.cpp  Modbus RTU slave (FC3, FC6, FC16)
tests/host/                 PC tests with Arduino stubs (run_host_tests.sh)
TESTING.md                  host test results, hardware bench procedure, record sheet
```

## How it works

| Task | Where | Rate |
|---|---|---|
| NTC ADC sample (non-blocking, start/read conversion) | Timer2 compare ISR | 1 kHz, 1000-sample average -> 1 s |
| Temperature math -> holding register | `loop()` | once per second |
| Modbus request handling | `loop()` -> `modbus_update()` | every pass, must stay < ~3 ms |

The ISR does no division, no float and no `analogRead()`, so `loop()` and the
UART receive interrupt are never starved. The ADC sum is copied with interrupts
disabled. The master always reads precomputed registers.

## Register map (holding registers, FC3)

Numbering is kept from the original vibration + temperature firmware, so
existing master configurations keep working. Positions no longer used are
reserved and read 0. New registers are appended after the slave ID register
(22, 23, ...), so existing addresses never move.

| Addr | Register | Unit / meaning |
|---|---|---|
| 0 | Reserved | reads 0 (was X axis RMS [0.01 g]) |
| 1 | Reserved | reads 0 (was Y axis RMS [0.01 g]) |
| 2 | Reserved | reads 0 (was Z axis RMS [0.01 g]) |
| 3 | Temperature | 0.01 degC (signed); fault: `-32768` (0x8000) NTC open, `32767` (0x7FFF) NTC short |
| 4 | Reserved | reads 0 (was X axis positive peak [0.01 g]) |
| 5 | Reserved | reads 0 (was Y axis positive peak [0.01 g]) |
| 6 | Reserved | reads 0 (was Z axis positive peak [0.01 g]) |
| 7 | Reserved | reads 0 (was X axis RMS [0.01 m/s2]) |
| 8 | Reserved | reads 0 (was Y axis RMS [0.01 m/s2]) |
| 9 | Reserved | reads 0 (was Z axis RMS [0.01 m/s2]) |
| 10 | Reserved | reads 0 (was X axis positive peak [0.01 m/s2]) |
| 11 | Reserved | reads 0 (was Y axis positive peak [0.01 m/s2]) |
| 12 | Reserved | reads 0 (was Z axis positive peak [0.01 m/s2]) |
| 13 | Reserved | reads 0 (was X axis negative peak [0.01 g]) |
| 14 | Reserved | reads 0 (was Y axis negative peak [0.01 g]) |
| 15 | Reserved | reads 0 (was Z axis negative peak [0.01 g]) |
| 16 | Reserved | reads 0 (was X axis negative peak [0.01 m/s2]) |
| 17 | Reserved | reads 0 (was Y axis negative peak [0.01 m/s2]) |
| 18 | Reserved | reads 0 (was Z axis negative peak [0.01 m/s2]) |
| 19 | Reserved | reads 0 (was ADXL ISR time (looptime)) |
| 20 | Longest `loop()` pass in the last second | us |
| 21 | Slave ID (read/write, FC6 or FC16) | 1-247 |
| 22 | Diagnostic status | bit0 NTC open, bit1 NTC short, bit2 last slave ID save failed |
| 23 | ADC average over the last second | 0.1 count (0-10230) |
| 24 | Frames seen on the bus (any slave ID) | count, wraps at 65535 |
| 25 | Valid requests for this slave | count, wraps |
| 26 | Requests for this slave with bad CRC | count, wraps |
| 27 | Buffer overflow / malformed frames | count, wraps |
| 28 | Exception replies sent | count, wraps |
| 29 | Uptime, high word | s |
| 30 | Uptime, low word (uptime = reg29 * 65536 + reg30) | s |
| 31 | Boot count (power-ups / resets, kept in EEPROM) | count, stops at 65534 |
| 32 | Longest sampling ISR run in the last second | 0.1 us |
| 33 | Longest sampling ISR run since power-up | 0.1 us |

All 34 registers (0-33) can be read in one FC3 request (up to 61 per request).
New registers are appended from 34 on.

ISR time is measured with Timer1 running free at 16 MHz (62.5 ns resolution),
from the first to the last statement of the Timer2 ISR body; register
save/restore adds about 2-3 us. CPU load of the sampler is roughly
reg32 / 10 us x 1000 per second, e.g. 50 (5.0 us) = 0.5 %.

### Reading the interface diagnostics

- Bus frames rising but valid requests not: the master polls other IDs only, or this slave's ID is wrong.
- CRC errors rising: wiring, termination, bias or baud/parity mismatch.
- Frame errors rising: frames merging (`loop()` too slow, see register 20) or a master sending oversized frames.
- Boot count rising unexpectedly: brown-outs or resets (check supply and BOD fuse).

Only register 21 is writable. Broadcast writes are ignored.

### Sensor fault detection

Every ADC sample is checked: `>= 1020` counts means the NTC is open (would be
below -63 degC), `<= 3` counts means it is shorted (would be above 259 degC).
If more than 10 samples (1%) of a 1 s window sit on a rail, register 3 reports
the fault value instead of a temperature, so loose/intermittent contacts are
caught too. The valid range -63 ... 259 degC is far outside busduct conditions.
The fault clears on the first clean 1 s window.

## Heartbeat LED (D13, onboard "L")

| LED | Meaning |
|---|---|
| Slow blink, 1 Hz (0.5 s on / 0.5 s off) | Normal: loop running, new temperature every second |
| Fast blink, 4 Hz | Running, but NTC open or shorted (register 3 shows the fault value) |
| Steady off or steady on | Sampling stopped, firmware hung, or controller in reset |
| A few quick flashes, then normal | Controller restarted (bootloader flashes D13 on reset) |

The LED is driven from `loop()` and blinks only while the 1 s sampling windows
keep arriving, so a hung loop freezes it and a stopped sampler turns it off.
The old firmware's "SPI clock" LED on the same pin no longer exists: this
firmware does not use SPI.

## Changing the slave ID

1. Write the new ID to register 21, addressed to the old ID (ack comes from the old ID).
2. Wait ~50 ms (EEPROM write + verify).
3. Read register 21 at the new ID. If there is no answer and the old ID still
   reports the old value, the save failed and was rolled back.

The ID is stored twice in EEPROM with an inverted check byte and verified by
read-back; it survives power loss, including power loss during the write.
Devices running the old firmware keep their ID when updated (legacy layout is migrated).

## Build options (top of the sketch)

- `USE_WATCHDOG` - 2 s watchdog. Only with the Optiboot bootloader; the old Nano bootloader hangs after a WDT reset.

## Hardware notes

- Ratiometric: feed the divider (100.5 kohm -> A0 -> NTC -> GND) from the same 5 V node as the AREF pin. The sketch uses `analogReference(EXTERNAL)`; never call `analogReference(DEFAULT/INTERNAL)` while AREF is driven.
- 100 nF from A0 to GND (NTC divider source impedance is ~50 kohm) and on AREF.
- RS485: pin 3 to both DE and /RE, 120 ohm termination at both bus ends, bias resistors.
- For EEPROM safety at 16 MHz set the brown-out detector to 4.3 V (extended fuse `0xFC`, needs ISP).
- Do not use `tone()` or PWM on pins 3/11: Timer2 is used for NTC sampling.
- Do not use PWM on pins 9/10 or the Servo library: Timer1 is the ISR timing counter.
- Do not call `analogRead()` anywhere: the Timer2 ISR owns the ADC.
- D13 is the heartbeat LED output; keep it free.
