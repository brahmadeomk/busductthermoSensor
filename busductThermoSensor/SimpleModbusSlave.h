#ifndef SIMPLE_MODBUS_SLAVE_H
#define SIMPLE_MODBUS_SLAVE_H

// SimpleModbusSlaveV10 (modified for busduct thermo/vibration sensor)

/*
 Modbus RTU slave implementing functions 3, 6 and 16.

 Framing: bytes are collected without blocking; a frame is complete after
 3.5 character times of bus silence (T3_5). modbus_update() must therefore
 be called from loop() at least every ~3 ms at 9600 baud, otherwise
 back-to-back frames on a multi-drop bus can merge and be missed.

 Function 3:  read holding registers (max 61 per request, 128-byte buffer).
 Function 6:  write single register  - only the writable register (see below).
 Function 16: write multiple registers - only the writable register, count 1.
 The writable register defaults to the last one; change it with
 modbus_set_writable_register().
 Broadcast (ID 0) writes are ignored; broadcast reads are not allowed.

 Frames addressed to other slaves and frames with a bad CRC are ignored
 silently. modbus_update() returns the total error count since start.

 The library never calls sketch code: update the register array from
 loop() when new measurements are ready.

 References:
 http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
*/

#include "Arduino.h"

// Interface counters since start-up; each wraps at 65535
struct ModbusStats
{
  unsigned int busFrames;   // every complete frame seen on the bus (any slave ID)
  unsigned int requests;    // frames for this slave (or broadcast) with a valid CRC
  unsigned int crcErrors;   // frames for this slave with a bad CRC
  unsigned int frameErrors; // receive buffer overflow or malformed FC6 / FC16 frame
  unsigned int exceptions;  // exception replies (illegal function / address / value)
};

// Opens the serial port, sets slave ID and RS485 driver-enable pin
void modbus_configure(HardwareSerial *SerialPort,
                      long baud,
                      unsigned char byteFormat,
                      unsigned char _slaveID,
                      unsigned char _TxEnablePin,
                      unsigned int _holdingRegsSize,
                      unsigned int* _regs);

// Selects the only register the master may write (FC6 / FC16)
void modbus_set_writable_register(unsigned int reg);

// Changes slave ID / frame timing without reopening the port
void modbus_update_comms(long baud, unsigned char byteFormat, unsigned char _slaveID);

// Interface counters, see ModbusStats
const ModbusStats& modbus_stats();

// Call from loop() as often as possible; returns total error count
unsigned int modbus_update();

#endif
