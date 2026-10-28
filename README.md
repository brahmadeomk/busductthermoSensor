# busductthermoSensor

Arduino Nano (ATmega328P, 16 MHz) Modbus RTU slave for busduct monitoring:
NTC temperature on A0, ratiometric with external AREF (5 V).

## Files

```
busductThermoSensor/
  busductThermoSensor.ino   sketch: sampling, register update, slave ID in EEPROM
  SimpleModbusSlave.h/.cpp  Modbus RTU slave (FC3, FC6, FC16)
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

Only register 21 is writable. Broadcast writes are ignored.

### Sensor fault detection

Every ADC sample is checked: `>= 1020` counts means the NTC is open (would be
below -63 degC), `<= 3` counts means it is shorted (would be above 259 degC).
If more than 10 samples (1%) of a 1 s window sit on a rail, register 3 reports
the fault value instead of a temperature, so loose/intermittent contacts are
caught too. The valid range -63 ... 259 degC is far outside busduct conditions.
The fault clears on the first clean 1 s window.

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
- Do not call `analogRead()` anywhere: the Timer2 ISR owns the ADC.
