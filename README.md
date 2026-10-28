# busductthermoSensor

Arduino Nano (ATmega328P, 16 MHz) Modbus RTU slave for busduct monitoring:
NTC temperature on A0 and 3-axis vibration from an ADXL345 over SPI.

## Files

```
busductThermoSensor/
  busductThermoSensor.ino   sketch: sampling, register update, slave ID in EEPROM
  SimpleModbusSlave.h/.cpp  Modbus RTU slave (FC3, FC6, FC16)
  SparkFun_ADXL345-master/  your modified ADXL345 library (not in this repo, copy it here)
```

## How it works

| Task | Where | Rate |
|---|---|---|
| NTC ADC sample (non-blocking, start/read conversion) | Timer2 compare ISR | 1 kHz, 1000-sample average -> 1 s |
| ADXL345 read, accumulate sum / sum^2 / min / max | INT0 ISR (DATA_READY) | 1600 Hz, 1600-sample window -> 1 s |
| RMS, peaks, temperature math -> holding registers | `loop()` | once per window |
| Modbus request handling | `loop()` -> `modbus_update()` | every pass, must stay < ~3 ms |

The ISRs do no division, no float and no `analogRead()`, so `loop()` and the
UART receive interrupt are never starved. Values shared with ISRs are copied
with interrupts disabled. The master always reads precomputed registers.

## Register map (holding registers, FC3)

| Addr | Name | Unit |
|---|---|---|
| 0-2 | X/Y/Z RMS | 0.01 g |
| 3 | Temperature | 0.01 degC (signed) |
| 4-6 | X/Y/Z positive peak | 0.01 g |
| 7-9 | X/Y/Z RMS | 0.01 m/s2 |
| 10-12 | X/Y/Z positive peak | 0.01 m/s2 |
| 13-15 | X/Y/Z negative peak | 0.01 g (signed) |
| 16-18 | X/Y/Z negative peak | 0.01 m/s2 (signed) |
| 19 | Last ADXL ISR duration | us |
| 20 | Longest `loop()` pass in last window | us |
| 21 | Slave ID (read/write, FC6 or FC16) | 1-247 |

Only register 21 is writable. Broadcast writes are ignored.

## Changing the slave ID

1. Write the new ID to register 21, addressed to the old ID (ack comes from the old ID).
2. Wait ~50 ms (EEPROM write + verify).
3. Read register 21 at the new ID. If there is no answer and the old ID still
   reports the old value, the save failed and was rolled back.

The ID is stored twice in EEPROM with an inverted check byte and verified by
read-back; it survives power loss, including power loss during the write.
Devices running the old firmware keep their ID when updated (legacy layout is migrated).

## Build options (top of the sketch)

- `ENABLE_TRIPLE_RESET` - three power cycles within 5 s restore ID 71.
- `USE_WATCHDOG` - 2 s watchdog. Only with the Optiboot bootloader; the old Nano bootloader hangs after a WDT reset.

## Hardware notes

- 3.0 V on AREF; the sketch uses `analogReference(EXTERNAL)`. Never call `analogReference(DEFAULT/INTERNAL)`.
- 100 nF from A0 to GND (NTC divider source impedance is ~50 kohm) and on AREF.
- RS485: pin 3 to both DE and /RE, 120 ohm termination at both bus ends, bias resistors.
- For EEPROM safety at 16 MHz set the brown-out detector to 4.3 V (extended fuse `0xFC`, needs ISP).
- Do not use `tone()` or PWM on pins 3/11: Timer2 is used for NTC sampling.
