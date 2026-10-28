// Busduct thermo sensor - Arduino Nano (ATmega328P, 16 MHz)
//
// - NTC on A0 sampled every 1 ms by Timer2 ISR, averaged over 1000 samples (1 s)
// - Ratiometric: divider supply and AREF are the same 5 V node, so the
//   reference voltage cancels and does not appear in the calculation
// - Open / shorted NTC is reported as a fixed fault value in the temperature register
// - The temperature register is refreshed in loop() once per second and is
//   served by modbus_update() on every master request
// - Slave ID is writable over Modbus (last register) and kept in EEPROM
//   as two verified copies, so it survives power loss

#include <EEPROM.h>
#include <avr/wdt.h>
#include "SimpleModbusSlave.h"

#define RTD_PIN            A0
#define TX_ENABLE_PIN      3
#define BAUD               9600
#define DEFAULT_SLAVE_ID   71

#define TEMP_SAMPLES       1000  // 1 ms each -> 1 s average

// Sensor fault detection, per ADC sample (ratiometric counts, independent of AREF voltage)
#define ADC_OPEN_COUNTS    1020  // >= : NTC open / disconnected (would be below -63 degC)
#define ADC_SHORT_COUNTS   3     // <= : NTC shorted (would be above 259 degC)
#define FAULT_MIN_SAMPLES  10    // > 1% of a window on a rail -> fault (also catches loose contacts)

#define TEMP_FAULT_OPEN    ((unsigned int)0x8000) // -32768 in register 3
#define TEMP_FAULT_SHORT   ((unsigned int)0x7FFF) //  32767 in register 3

#define USE_WATCHDOG        0    // 1 only with Optiboot (old Nano bootloader hangs after WDT reset)

// NTC: fixed resistor to AREF (5 V), NTC to GND, junction on A0
const float R_FIXED  = 100500.0f; // [ohm]
const float NTC_R0   = 100000.0f; // [ohm] at T0
const float NTC_T0   = 298.15f;   // [K]
const float NTC_BETA = 3950.0f;   // [K]

// Register map: numbering kept from the original vibration + temperature firmware,
// so masters need no reconfiguration. Positions no longer used are Reserved_xx and read 0.
// New registers are appended after SlaveIDReg (22, 23, ...), so existing
// addresses never move. SlaveIDReg is the only writable register.
enum
{
  Reserved_00,    //  0  reads 0 (was X axis RMS [0.01 g])
  Reserved_01,    //  1  reads 0 (was Y axis RMS [0.01 g])
  Reserved_02,    //  2  reads 0 (was Z axis RMS [0.01 g])
  TempValue,      //  3  0.01 degC (signed); -32768 = NTC open, 32767 = NTC short
  Reserved_04,    //  4  reads 0 (was X axis positive peak [0.01 g])
  Reserved_05,    //  5  reads 0 (was Y axis positive peak [0.01 g])
  Reserved_06,    //  6  reads 0 (was Z axis positive peak [0.01 g])
  Reserved_07,    //  7  reads 0 (was X axis RMS [0.01 m/s2])
  Reserved_08,    //  8  reads 0 (was Y axis RMS [0.01 m/s2])
  Reserved_09,    //  9  reads 0 (was Z axis RMS [0.01 m/s2])
  Reserved_10,    // 10  reads 0 (was X axis positive peak [0.01 m/s2])
  Reserved_11,    // 11  reads 0 (was Y axis positive peak [0.01 m/s2])
  Reserved_12,    // 12  reads 0 (was Z axis positive peak [0.01 m/s2])
  Reserved_13,    // 13  reads 0 (was X axis negative peak [0.01 g])
  Reserved_14,    // 14  reads 0 (was Y axis negative peak [0.01 g])
  Reserved_15,    // 15  reads 0 (was Z axis negative peak [0.01 g])
  Reserved_16,    // 16  reads 0 (was X axis negative peak [0.01 m/s2])
  Reserved_17,    // 17  reads 0 (was Y axis negative peak [0.01 m/s2])
  Reserved_18,    // 18  reads 0 (was Z axis negative peak [0.01 m/s2])
  Reserved_19,    // 19  reads 0 (was ADXL ISR time (looptime))
  Modbuslooptime, // 20  longest loop() pass in the last second [us], must stay < ~3000
  SlaveIDReg,     // 21  1-247, read/write
  // add new registers here (22, 23, ...)
  HOLDING_REGS_SIZE
};

static_assert(TempValue == 3, "register map changed");
static_assert(Modbuslooptime == 20, "register map changed");
static_assert(SlaveIDReg == 21, "register map changed");

unsigned int holdingRegs[HOLDING_REGS_SIZE];

float rInf; // NTC R at infinite temperature, from beta model

// ---------------------------------------------------------------------------
// NTC sampling (Timer2, 1 kHz)
// ---------------------------------------------------------------------------
volatile uint32_t adcSumReady = 0;
volatile uint16_t openCountReady = 0;
volatile uint16_t shortCountReady = 0;
volatile bool tempReady = false;

void tempSamplerInit()
{
  TCCR2A = _BV(WGM21);            // CTC
  TCCR2B = _BV(CS22) | _BV(CS20); // /128 -> 125 kHz
  OCR2A  = 124;                   // 1 kHz
  TIMSK2 = _BV(OCIE2A);
  ADCSRA |= _BV(ADSC);            // first conversion
}

ISR(TIMER2_COMPA_vect)
{
  static uint32_t acc = 0;
  static uint16_t count = 0;
  static uint16_t openCount = 0;
  static uint16_t shortCount = 0;

  if (!(ADCSRA & _BV(ADSC))) // conversion started on previous tick is done (~104 us)
  {
    uint16_t v = ADC;
    acc += v;
    if (v >= ADC_OPEN_COUNTS)
      openCount++;
    else if (v <= ADC_SHORT_COUNTS)
      shortCount++;

    if (++count >= TEMP_SAMPLES)
    {
      adcSumReady = acc;
      openCountReady = openCount;
      shortCountReady = shortCount;
      tempReady = true;
      acc = 0;
      count = 0;
      openCount = 0;
      shortCount = 0;
    }
  }
  ADCSRA |= _BV(ADSC); // start next conversion
}

// ---------------------------------------------------------------------------
// Register update
// ---------------------------------------------------------------------------
static unsigned int toReg(float v)
{
  long r = lround(v);
  if (r > 32767) r = 32767;
  if (r < -32768) r = -32768;
  return (unsigned int)(int16_t)r; // signed value, master reads as INT16
}

void processTemperature(unsigned long maxPassMicros)
{
  noInterrupts();
  uint32_t sum = adcSumReady;
  uint16_t openCount = openCountReady;
  uint16_t shortCount = shortCountReady;
  tempReady = false;
  interrupts();

  if (openCount > FAULT_MIN_SAMPLES)
    holdingRegs[TempValue] = TEMP_FAULT_OPEN;
  else if (shortCount > FAULT_MIN_SAMPLES)
    holdingRegs[TempValue] = TEMP_FAULT_SHORT;
  else
  {
    float ratio = (sum / (float)TEMP_SAMPLES) / 1024.0f; // = R_ntc / (R_fixed + R_ntc)
    ratio = constrain(ratio, 0.001f, 0.999f);             // keeps log() finite
    float rNtc = R_FIXED * ratio / (1.0f - ratio);
    float tC = NTC_BETA / log(rNtc / rInf) - 273.15f;
    holdingRegs[TempValue] = toReg(tC * 100.0f);
  }

  holdingRegs[Modbuslooptime] = maxPassMicros > 65535UL ? 65535U : (unsigned int)maxPassMicros;
}

// ---------------------------------------------------------------------------
// Slave ID in EEPROM
// ---------------------------------------------------------------------------
// EEPROM layout
//  0..2 : legacy layout [magicLo, magicHi, slaveId], read only for migration
// 16..19: slave ID copy A [magicLo, magicHi, id, ~id]
// 24..27: slave ID copy B [magicLo, magicHi, id, ~id]
const uint16_t CONFIG_MAGIC = 0xA5A5;
const int LEGACY_ADDR       = 0;
const int ID_COPY_A         = 16;
const int ID_COPY_B         = 24;

uint8_t currentSlaveId = DEFAULT_SLAVE_ID;

static bool validId(unsigned int id)
{
  return id >= 1 && id <= 247;
}

static bool readIdCopy(int addr, uint8_t &id)
{
  uint16_t magic = EEPROM.read(addr) | ((uint16_t)EEPROM.read(addr + 1) << 8);
  uint8_t v   = EEPROM.read(addr + 2);
  uint8_t inv = EEPROM.read(addr + 3);
  if (magic != CONFIG_MAGIC || (uint8_t)~v != inv || !validId(v))
    return false;
  id = v;
  return true;
}

static bool writeIdCopy(int addr, uint8_t id)
{
  EEPROM.update(addr + 0, lowByte(CONFIG_MAGIC));
  EEPROM.update(addr + 1, highByte(CONFIG_MAGIC));
  EEPROM.update(addr + 2, id);
  EEPROM.update(addr + 3, (uint8_t)~id);
  uint8_t check;
  return readIdCopy(addr, check) && check == id; // verify by read-back
}

// Copy A is written first: if power fails mid-write, B still holds a valid ID
bool saveSlaveId(uint8_t id)
{
  bool okA = writeIdCopy(ID_COPY_A, id);
  bool okB = writeIdCopy(ID_COPY_B, id);
  return okA && okB;
}

uint8_t loadSlaveId()
{
  uint8_t idA, idB;
  bool a = readIdCopy(ID_COPY_A, idA);
  bool b = readIdCopy(ID_COPY_B, idB);

  if (a)
  {
    if (!b || idB != idA)
      writeIdCopy(ID_COPY_B, idA); // finish an interrupted save
    return idA;
  }
  if (b)
  {
    writeIdCopy(ID_COPY_A, idB); // repair A
    return idB;
  }

  // Migrate from the old single-copy layout (devices already in the field)
  uint16_t magic = EEPROM.read(LEGACY_ADDR) | ((uint16_t)EEPROM.read(LEGACY_ADDR + 1) << 8);
  uint8_t legacyId = EEPROM.read(LEGACY_ADDR + 2);
  uint8_t id = (magic == CONFIG_MAGIC && validId(legacyId)) ? legacyId : DEFAULT_SLAVE_ID;
  saveSlaveId(id);
  return id;
}

void checkModbusSlaveIdUpdate()
{
  unsigned int requestedId = holdingRegs[SlaveIDReg];
  if (requestedId == currentSlaveId)
    return;

  if (validId(requestedId))
  {
    // Switch only once the new ID is safely stored, so the ID used now
    // is always the ID used after the next power-up
    if (saveSlaveId((uint8_t)requestedId))
    {
      currentSlaveId = (uint8_t)requestedId;
      modbus_update_comms(BAUD, SERIAL_8N1, currentSlaveId);
    }
    else
      saveSlaveId(currentSlaveId); // roll back, EEPROM write failed
  }
  holdingRegs[SlaveIDReg] = currentSlaveId; // master reads back the ID actually in use
}

// ---------------------------------------------------------------------------
void setup()
{
#if USE_WATCHDOG
  wdt_disable();
#endif

  analogReference(EXTERNAL); // 5 V on AREF pin - never switch to DEFAULT/INTERNAL
  analogRead(RTD_PIN);       // sets ADMUX (reference + channel) for the Timer2 sampler

  currentSlaveId = loadSlaveId();

  modbus_configure(&Serial, BAUD, SERIAL_8N1, currentSlaveId, TX_ENABLE_PIN,
                   HOLDING_REGS_SIZE, holdingRegs);
  modbus_set_writable_register(SlaveIDReg);
  holdingRegs[SlaveIDReg] = currentSlaveId;

  rInf = NTC_R0 * exp(-NTC_BETA / NTC_T0);

  tempSamplerInit();

#if USE_WATCHDOG
  wdt_enable(WDTO_2S);
#endif
}

void loop()
{
  static unsigned long lastPassMicros = micros();
  static unsigned long maxPassMicros = 0;

  unsigned long now = micros();
  unsigned long pass = now - lastPassMicros;
  lastPassMicros = now;
  if (pass > maxPassMicros)
    maxPassMicros = pass;

  modbus_update();
  checkModbusSlaveIdUpdate(); // right after modbus_update, before anything touches holdingRegs

  if (tempReady)
  {
    processTemperature(maxPassMicros);
    maxPassMicros = 0;
  }

#if USE_WATCHDOG
  wdt_reset();
#endif
}
