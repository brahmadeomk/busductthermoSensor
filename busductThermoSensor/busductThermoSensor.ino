// Busduct thermo / vibration sensor - Arduino Nano (ATmega328P, 16 MHz)
//
// - NTC on A0 sampled every 1 ms by Timer2 ISR, averaged over 1000 samples (1 s)
// - ADXL345 (SPI) DATA_READY on INT0, 1600 samples per window (1 s at 1600 Hz ODR)
//   ISR only accumulates sum / sum of squares / min / max; math runs in loop()
// - Holding registers are refreshed in loop() when a window completes and are
//   served by modbus_update() on every master request
// - Slave ID is writable over Modbus (last register) and kept in EEPROM
//   as two verified copies, so it survives power loss

#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "SimpleModbusSlave.h"
#include "SparkFun_ADXL345-master/mFFT_SparkFun_ADXL345.cpp"

#define RTD_PIN            A0
#define TX_ENABLE_PIN      3
#define ADXL_CS_PIN        4
#define ADXL_INT1_PIN      2   // INT0, ADXL345 INT1 output
#define BAUD               9600
#define DEFAULT_SLAVE_ID   71

#define ADXL345_INT_DATA_READY_BIT 7

#define TEMP_SAMPLES       1000  // 1 ms each -> 1 s average
#define VIB_SAMPLES        1600  // at 1600 Hz ODR -> 1 s window

#define ENABLE_TRIPLE_RESET 0    // 1: three power cycles within 5 s restore DEFAULT_SLAVE_ID
#define RESET_WINDOW_MS     5000
#define USE_WATCHDOG        0    // 1 only with Optiboot (old Nano bootloader hangs after WDT reset)

// NTC: fixed resistor to AREF (3.0 V), NTC to GND, junction on A0
const float R_FIXED  = 100500.0f; // [ohm]
const float NTC_R0   = 100000.0f; // [ohm] at T0
const float NTC_T0   = 298.15f;   // [K]
const float NTC_BETA = 3950.0f;   // [K]

// ADXL345 full resolution: 3.9 mg/LSB -> register unit is 0.01 g
const float LSB_TO_CG          = 0.39f;
const float G_TO_MS2           = 9.80665f;
const float RMS_ZERO_THRESHOLD = 0.1f; // [LSB]

// Register map (do not reorder: SlaveIDReg must stay last, it is the only writable one)
enum
{
  XaxisRMS, YaxisRMS, ZaxisRMS,                         // 0.01 g
  TempValue,                                            // 0.01 degC
  XaxismodPosP, YaxismodPosP, ZaxismodPosP,             // 0.01 g
  XaxisRMS_m_s, YaxisRMS_m_s, ZaxisRMS_m_s,             // 0.01 m/s2
  XaxismodPosP_m_s, YaxismodPosP_m_s, ZaxismodPosP_m_s, // 0.01 m/s2
  XaxismodNegP, YaxismodNegP, ZaxismodNegP,             // 0.01 g
  XaxismodNegP_m_s, YaxismodNegP_m_s, ZaxismodNegP_m_s, // 0.01 m/s2
  looptime,       // last ADXL ISR duration [us]
  Modbuslooptime, // longest loop() pass in the last window [us], must stay < ~3000
  SlaveIDReg,
  HOLDING_REGS_SIZE
};

unsigned int holdingRegs[HOLDING_REGS_SIZE];

ADXL345 adxl = ADXL345(ADXL_CS_PIN);

float rInf; // NTC R at infinite temperature, from beta model

// ---------------------------------------------------------------------------
// NTC sampling (Timer2, 1 kHz)
// ---------------------------------------------------------------------------
volatile uint32_t adcSumReady = 0;
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

  if (!(ADCSRA & _BV(ADSC))) // conversion started on previous tick is done (~104 us)
  {
    acc += ADC;
    if (++count >= TEMP_SAMPLES)
    {
      adcSumReady = acc;
      tempReady = true;
      acc = 0;
      count = 0;
    }
  }
  ADCSRA |= _BV(ADSC); // start next conversion
}

// ---------------------------------------------------------------------------
// Vibration sampling (ADXL345 DATA_READY on INT0)
// ---------------------------------------------------------------------------
struct AxisAcc
{
  int32_t  sum;
  uint64_t sumSq;
  int16_t  minV;
  int16_t  maxV;
};

// accIsr is touched only by the ISR. accReady is written by the ISR and read by
// loop() with interrupts disabled (cli() is a compiler memory barrier on AVR).
static AxisAcc accIsr[3];
static AxisAcc accReady[3];
static uint16_t vibCount = 0;
volatile bool vibReady = false;
volatile uint8_t adxlSampleTick = 0;
volatile unsigned int adxlIsrMicros = 0;

static void resetAcc(AxisAcc *acc)
{
  for (uint8_t a = 0; a < 3; a++)
  {
    acc[a].sum = 0;
    acc[a].sumSq = 0;
    acc[a].minV = 32767;
    acc[a].maxV = -32768;
  }
}

void adxlISR()
{
  unsigned long t0 = micros();
  int v[3];

  // Reading the data registers also clears DATA_READY
  adxl.readAccel(&v[0], &v[1], &v[2]);

  for (uint8_t a = 0; a < 3; a++)
  {
    int16_t s = v[a];
    accIsr[a].sum += s;
    accIsr[a].sumSq += (uint32_t)((int32_t)s * s);
    if (s < accIsr[a].minV) accIsr[a].minV = s;
    if (s > accIsr[a].maxV) accIsr[a].maxV = s;
  }
  adxlSampleTick++;

  if (++vibCount >= VIB_SAMPLES)
  {
    memcpy(accReady, accIsr, sizeof(accIsr));
    resetAcc(accIsr);
    vibCount = 0;
    vibReady = true;
  }
  adxlIsrMicros = micros() - t0;
}

// INT1 is edge triggered: if one edge is missed DATA_READY stays high and no new
// edge ever comes. Reading the data once clears it and restarts the interrupts.
void recoverAdxlIfStalled()
{
  static uint8_t lastTick = 0;
  static unsigned long lastCheck = 0;

  if (millis() - lastCheck < 100)
    return;
  lastCheck = millis();

  if (adxlSampleTick == lastTick)
  {
    int d[3];
    noInterrupts();
    adxl.readAccel(&d[0], &d[1], &d[2]);
    interrupts();
  }
  lastTick = adxlSampleTick;
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

void processTemperature()
{
  noInterrupts();
  uint32_t sum = adcSumReady;
  tempReady = false;
  interrupts();

  float ratio = (sum / (float)TEMP_SAMPLES) / 1024.0f;
  ratio = constrain(ratio, 0.001f, 0.999f); // open / short sensor guard
  float rNtc = R_FIXED * ratio / (1.0f - ratio);
  float tC = NTC_BETA / log(rNtc / rInf) - 273.15f;

  holdingRegs[TempValue] = toReg(tC * 100.0f);
}

void processVibration(unsigned long maxPassMicros)
{
  AxisAcc acc[3];
  noInterrupts();
  memcpy(acc, accReady, sizeof(acc));
  vibReady = false;
  unsigned int isrMicros = adxlIsrMicros;
  interrupts();

  const float n = VIB_SAMPLES;
  for (uint8_t a = 0; a < 3; a++)
  {
    float mean = acc[a].sum / n;

    // N^2 * variance computed exactly in 64-bit integers
    int64_t varN2 = (int64_t)acc[a].sumSq * VIB_SAMPLES - (int64_t)acc[a].sum * acc[a].sum;
    float rms = (varN2 > 0) ? sqrt((float)varN2) / n : 0.0f;
    if (rms <= RMS_ZERO_THRESHOLD)
      rms = 0.0f;

    float rmsCg = rms * LSB_TO_CG;
    float posCg = (acc[a].maxV - mean) * LSB_TO_CG;
    float negCg = (acc[a].minV - mean) * LSB_TO_CG;

    holdingRegs[XaxisRMS + a]         = toReg(rmsCg);
    holdingRegs[XaxismodPosP + a]     = toReg(posCg);
    holdingRegs[XaxismodNegP + a]     = toReg(negCg);
    holdingRegs[XaxisRMS_m_s + a]     = toReg(rmsCg * G_TO_MS2);
    holdingRegs[XaxismodPosP_m_s + a] = toReg(posCg * G_TO_MS2);
    holdingRegs[XaxismodNegP_m_s + a] = toReg(negCg * G_TO_MS2);
  }

  holdingRegs[looptime] = isrMicros;
  holdingRegs[Modbuslooptime] = maxPassMicros > 65535UL ? 65535U : (unsigned int)maxPassMicros;
}

// ---------------------------------------------------------------------------
// Slave ID in EEPROM
// ---------------------------------------------------------------------------
// EEPROM layout
//  0..4 : legacy layout [magicLo, magicHi, slaveId, bootCounter, bootMarker]
//         bytes 3,4 are still used by the triple power-cycle reset
// 16..19: slave ID copy A [magicLo, magicHi, id, ~id]
// 24..27: slave ID copy B [magicLo, magicHi, id, ~id]
const uint16_t CONFIG_MAGIC = 0xA5A5;
const int LEGACY_ADDR       = 0;
const int BOOT_COUNTER_ADDR = 3;
const int BOOT_MARKER_ADDR  = 4;
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

#if ENABLE_TRIPLE_RESET
unsigned long bootStartMillis = 0;
bool windowCleared = false;

void handleTripleResetOnBoot()
{
  uint8_t counter = EEPROM.read(BOOT_COUNTER_ADDR);
  if (EEPROM.read(BOOT_MARKER_ADDR) == 1)
    counter++;
  else
    counter = 1;

  if (counter >= 3)
  {
    saveSlaveId(DEFAULT_SLAVE_ID);
    currentSlaveId = DEFAULT_SLAVE_ID;
    EEPROM.update(BOOT_COUNTER_ADDR, 0);
    EEPROM.update(BOOT_MARKER_ADDR, 0);
  }
  else
  {
    EEPROM.update(BOOT_COUNTER_ADDR, counter);
    EEPROM.update(BOOT_MARKER_ADDR, 1);
  }
  bootStartMillis = millis();
  windowCleared = false;
}

void clearBootWindowIfExpired()
{
  if (!windowCleared && (millis() - bootStartMillis) >= RESET_WINDOW_MS)
  {
    EEPROM.update(BOOT_COUNTER_ADDR, 0);
    EEPROM.update(BOOT_MARKER_ADDR, 0);
    windowCleared = true;
  }
}
#endif

// ---------------------------------------------------------------------------
void setup()
{
#if USE_WATCHDOG
  wdt_disable();
#endif

  analogReference(EXTERNAL); // 3.0 V on AREF - never switch to DEFAULT/INTERNAL
  analogRead(RTD_PIN);       // sets ADMUX (reference + channel) for the Timer2 sampler

  currentSlaveId = loadSlaveId();
#if ENABLE_TRIPLE_RESET
  handleTripleResetOnBoot();
#endif

  modbus_configure(&Serial, BAUD, SERIAL_8N1, currentSlaveId, TX_ENABLE_PIN,
                   HOLDING_REGS_SIZE, holdingRegs);
  holdingRegs[SlaveIDReg] = currentSlaveId;

  rInf = NTC_R0 * exp(-NTC_BETA / NTC_T0);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  adxl.powerOn();
  adxl.setRangeSetting(16);
  adxl.setSpiBit(0);
  adxl.set_bw(ADXL345_BW_800); // 1600 Hz output data rate
  adxl.setFullResBit(1);
  adxl.setInterrupt(ADXL345_INT_DATA_READY_BIT, true);
  adxl.setInterruptMapping(ADXL345_INT_DATA_READY_BIT, ADXL345_INT1_PIN);

  resetAcc(accIsr);
  pinMode(ADXL_INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ADXL_INT1_PIN), adxlISR, RISING);

  // Clear a DATA_READY that went high before the interrupt was attached
  int d[3];
  noInterrupts();
  adxl.readAccel(&d[0], &d[1], &d[2]);
  interrupts();

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
    processTemperature();

  if (vibReady)
  {
    processVibration(maxPassMicros);
    maxPassMicros = 0;
  }

  recoverAdxlIfStalled();

#if ENABLE_TRIPLE_RESET
  clearBootWindowIfExpired();
#endif

#if USE_WATCHDOG
  wdt_reset();
#endif
}
