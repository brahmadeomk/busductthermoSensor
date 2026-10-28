#include "SimpleModbusSlave.h"

#define BUFFER_SIZE   128
#define MAX_READ_REGS ((BUFFER_SIZE - 5) / 2)   // 61 regs fit in one reply

// frame[] is used to receive and transmit packets
static unsigned char frame[BUFFER_SIZE];
static unsigned int holdingRegsSize;
static unsigned int writableReg;
static unsigned int* regs;
static unsigned char broadcastFlag;
static unsigned char slaveID;
static unsigned char function;
static unsigned char TxEnablePin;
static unsigned int errorCount;
static ModbusStats stats;
static unsigned int T3_5; // frame delay [us]
static HardwareSerial* ModbusPort;

static unsigned char rxLen = 0;
static unsigned char rxOverflow = 0;
static unsigned long lastRxMicros = 0;

static void processFrame(unsigned char len);
static void exceptionResponse(unsigned char exception);
static unsigned int calculateCRC(unsigned char bufferSize);
static void sendPacket(unsigned char bufferSize);

void modbus_configure(HardwareSerial *SerialPort,
                      long baud,
                      unsigned char byteFormat,
                      unsigned char _slaveID,
                      unsigned char _TxEnablePin,
                      unsigned int _holdingRegsSize,
                      unsigned int* _regs)
{
  ModbusPort = SerialPort;
  ModbusPort->begin(baud, byteFormat);
  modbus_update_comms(baud, byteFormat, _slaveID);
  holdingRegsSize = _holdingRegsSize;
  writableReg = _holdingRegsSize - 1;
  regs = _regs;
  TxEnablePin = _TxEnablePin;
  pinMode(TxEnablePin, OUTPUT);
  digitalWrite(TxEnablePin, LOW);
  errorCount = 0;
  memset(&stats, 0, sizeof(stats));
}

const ModbusStats& modbus_stats()
{
  return stats;
}

void modbus_set_writable_register(unsigned int reg)
{
  writableReg = reg;
}

void modbus_update_comms(long baud, unsigned char byteFormat, unsigned char _slaveID)
{
  (void)byteFormat;
  slaveID = _slaveID;
  rxLen = 0;
  rxOverflow = 0;

  // Above 19200 baud Modbus fixes the frame delay at 1.75 ms,
  // below it is 3.5 character times (11 bits per char incl. start/stop/parity slot)
  if (baud > 19200)
    T3_5 = 1750;
  else
    T3_5 = 35000000 / baud; // 3645 us at 9600
}

unsigned int modbus_update()
{
  // Drain the UART without blocking
  while (ModbusPort->available())
  {
    unsigned char b = ModbusPort->read();
    if (rxLen < BUFFER_SIZE)
      frame[rxLen++] = b;
    else
      rxOverflow = 1;
    lastRxMicros = micros();
  }

  if (rxLen == 0)
    return errorCount;

  // A frame is complete only after 3.5 char of bus silence
  if ((unsigned long)(micros() - lastRxMicros) < T3_5)
    return errorCount;

  unsigned char len = rxLen;
  rxLen = 0;
  stats.busFrames++;

  if (rxOverflow)
  {
    rxOverflow = 0;
    stats.frameErrors++;
    return ++errorCount;
  }

  // Shorter than any supported request (also other slaves' short replies)
  if (len < 8)
    return errorCount;

  processFrame(len);
  return errorCount;
}

static void processFrame(unsigned char len)
{
  unsigned char id = frame[0];
  broadcastFlag = (id == 0);

  if (id != slaveID && !broadcastFlag)
    return; // another slave's traffic

  unsigned int crc = ((unsigned int)frame[len - 2] << 8) | frame[len - 1];
  if (calculateCRC(len - 2) != crc)
  {
    stats.crcErrors++;
    errorCount++;
    return; // never answer a corrupt frame
  }
  stats.requests++;

  function = frame[1];
  unsigned int startingAddress = ((unsigned int)frame[2] << 8) | frame[3];
  unsigned int no_of_registers = ((unsigned int)frame[4] << 8) | frame[5];
  unsigned int crc16;

  if (function == 3)
  {
    if (broadcastFlag)
      return; // broadcast not allowed for reads

    if (no_of_registers == 0 || no_of_registers > MAX_READ_REGS)
    {
      exceptionResponse(3);
      return;
    }
    if (startingAddress >= holdingRegsSize ||
        no_of_registers > holdingRegsSize - startingAddress)
    {
      exceptionResponse(2);
      return;
    }

    unsigned char noOfBytes = no_of_registers * 2;
    unsigned char responseFrameSize = 5 + noOfBytes;
    frame[0] = slaveID;
    frame[1] = function;
    frame[2] = noOfBytes;
    unsigned char address = 3;

    for (unsigned int index = startingAddress; index < startingAddress + no_of_registers; index++)
    {
      unsigned int temp = regs[index];
      frame[address++] = temp >> 8;
      frame[address++] = temp & 0xFF;
    }

    crc16 = calculateCRC(responseFrameSize - 2);
    frame[responseFrameSize - 2] = crc16 >> 8;
    frame[responseFrameSize - 1] = crc16 & 0xFF;
    sendPacket(responseFrameSize);
  }
  else if (function == 6)
  {
    if (len != 8)
    {
      stats.frameErrors++;
      errorCount++;
      return;
    }
    if (broadcastFlag)
      return; // never accept broadcast writes (would re-address every slave)

    // Only one register (SlaveIDReg in the sketch) is writable
    if (startingAddress != writableReg || startingAddress >= holdingRegsSize)
    {
      exceptionResponse(2);
      return;
    }

    regs[startingAddress] = no_of_registers; // bytes 4..5 carry the value for FC6

    // FC6 reply is an exact echo; frame[0..7] still holds the request and its CRC
    sendPacket(8);
  }
  else if (function == 16)
  {
    unsigned char byteCount = frame[6];
    if (len < 9 || byteCount != len - 9 || byteCount != no_of_registers * 2)
    {
      stats.frameErrors++;
      errorCount++;
      return;
    }
    if (broadcastFlag)
      return;

    if (no_of_registers == 0 || no_of_registers > 123)
    {
      exceptionResponse(3);
      return;
    }
    if (startingAddress != writableReg || startingAddress >= holdingRegsSize || no_of_registers != 1)
    {
      exceptionResponse(2);
      return;
    }

    regs[startingAddress] = ((unsigned int)frame[7] << 8) | frame[8];

    // FC16 reply: first 6 bytes of the request + new CRC
    crc16 = calculateCRC(6);
    frame[6] = crc16 >> 8;
    frame[7] = crc16 & 0xFF;
    sendPacket(8);
  }
  else
    exceptionResponse(1);
}

static void exceptionResponse(unsigned char exception)
{
  stats.exceptions++;
  errorCount++;
  if (!broadcastFlag)
  {
    frame[0] = slaveID;
    frame[1] = (function | 0x80);
    frame[2] = exception;
    unsigned int crc16 = calculateCRC(3);
    frame[3] = crc16 >> 8;
    frame[4] = crc16 & 0xFF;
    sendPacket(5);
  }
}

static unsigned int calculateCRC(unsigned char bufferSize)
{
  unsigned int temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp ^= frame[i];
    for (unsigned char j = 0; j < 8; j++)
    {
      unsigned char flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Swap bytes: crcLo is sent first
  return ((temp << 8) | (temp >> 8)) & 0xFFFF;
}

static void sendPacket(unsigned char bufferSize)
{
  // 3.5 char of silence already elapsed while detecting end of request
  digitalWrite(TxEnablePin, HIGH);
  delayMicroseconds(50); // transceiver driver enable
  ModbusPort->write(frame, bufferSize);
  ModbusPort->flush(); // AVR core: returns after last stop bit has left the UART
  digitalWrite(TxEnablePin, LOW);

  // Discard local echo if RE is not tied to DE
  while (ModbusPort->available())
    ModbusPort->read();
}
