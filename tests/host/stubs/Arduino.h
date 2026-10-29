// Minimal Arduino / AVR stand-ins so the sketch and library compile on a PC.
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef uint8_t byte;
#define A0 14
#define OUTPUT 1
#define INPUT 0
#define LOW 0
#define HIGH 1
#define RISING 3
#define EXTERNAL 0
#define SERIAL_8N1 0x06
#define _BV(b) (1u<<(b))
extern volatile uint8_t TCCR2A,TCCR2B,OCR2A,TIMSK2,ADCSRA; extern volatile uint16_t ADC;
struct FakeTcnt { uint16_t v=0; uint16_t step=80; operator uint16_t(){ v+=step; return v; } };
extern FakeTcnt TCNT1; extern volatile uint8_t TCCR1A,TCCR1B;
#define CS10 0
#define WGM21 1
#define CS22 2
#define CS20 0
#define OCIE2A 1
#define ADSC 6
#define ISR(v) extern "C" void v(void)
#define constrain(a,l,h) ((a)<(l)?(l):((a)>(h)?(h):(a)))
#define lowByte(w) ((uint8_t)((w)&0xff))
#define highByte(w) ((uint8_t)((w)>>8))
#define digitalPinToInterrupt(p) ((p)==2?0:1)
unsigned long micros(); unsigned long millis(); void delayMicroseconds(unsigned int);
void pinMode(uint8_t,uint8_t); void digitalWrite(uint8_t,uint8_t); int analogRead(uint8_t);
void analogReference(uint8_t); void noInterrupts(); void interrupts();
void attachInterrupt(uint8_t, void(*)(), int);
class HardwareSerial { public: void begin(unsigned long, uint8_t); int available(); int read();
  size_t write(const uint8_t*, size_t); size_t write(uint8_t); void flush(); };
extern HardwareSerial Serial;
