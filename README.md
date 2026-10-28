# busductthermoSensor

Arduino Nano (ATmega328P, 16 MHz) Modbus RTU slave for busduct monitoring:
NTC temperature on A0.

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

Addresses are kept from the earlier vibration + temperature firmware, so
existing master configurations keep working.

| Addr | Name | Unit |
|---|---|---|
| 0-2 | unused (former vibration), reads 0 | |
| 3 | Temperature | 0.01 degC (signed) |
| 4-19 | unused (former vibration), reads 0 | |
| 20 | Longest `loop()` pass in the last second | us |
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
- Do not call `analogRead()` anywhere: the Timer2 ISR owns the ADC.
