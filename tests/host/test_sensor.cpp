// Host test: sketch sampling ISR, temperature conversion, fault detection,
// diagnostics registers and EEPROM slave ID handling with simulated ADC / Timer1.
#include "Arduino.h"
FakeTcnt TCNT1; volatile uint8_t TCCR1A,TCCR1B; volatile uint8_t TCCR2A,TCCR2B,OCR2A,TIMSK2,ADCSRA; volatile uint16_t ADC;
unsigned long micros(){return 0;} unsigned long fakeMillis = 0; unsigned long millis(){return fakeMillis;} void delayMicroseconds(unsigned int){}
void pinMode(uint8_t,uint8_t){} int ledState = 0, ledToggles = 0;
void digitalWrite(uint8_t pin,uint8_t v){ if(pin==13){ if(v!=ledState) ledToggles++; ledState=v; } } int analogRead(uint8_t){return 0;}
void analogReference(uint8_t){} void noInterrupts(){} void interrupts(){} void attachInterrupt(uint8_t,void(*)(),int){}
void HardwareSerial::begin(unsigned long,uint8_t){} int HardwareSerial::available(){return 0;} int HardwareSerial::read(){return -1;}
size_t HardwareSerial::write(const uint8_t*,size_t n){return n;} void HardwareSerial::flush(){}
HardwareSerial Serial;
#include "EEPROM.h"
uint8_t ee[1024]; uint8_t EEPROMClass::read(int a){return ee[a];} void EEPROMClass::update(int a,uint8_t v){ee[a]=v;} EEPROMClass EEPROM;
void wdt_enable(int){} void wdt_reset(){} void wdt_disable(){}
#include "busductThermoSensor.ino"
#include <cstdio>
#include <functional>
int fails=0;
int16_t window(std::function<uint16_t(int)> gen){
  for(int i=0;i<TEMP_SAMPLES;i++){ ADC=gen(i); ADCSRA=0; TIMER2_COMPA_vect(); }
  if(!tempReady){printf("no window\n");fails++;}
  processTemperature(0); return (int16_t)holdingRegs[TempValue];
}
void check(const char*n,int16_t got,int lo,int hi){bool ok=got>=lo&&got<=hi; printf("%-40s %6d %s\n",n,got,ok?"PASS":"FAIL"); if(!ok)fails++;}
int main(){
  for(int i=0;i<1024;i++) ee[i]=0xFF;
  setup();
  check("25 C (511 counts)", window([](int){return 511;}), 2490,2510);
  check("25 C dithered 510/511", window([](int i){return i%10<3?510:511;}), 2490,2510);
  check("100 C (66/67 counts)", window([](int i){return i%2?66:67;}), 9900,10100);
  check("0 C (788)", window([](int){return 788;}), -10,10);
  check("open NTC (1023)", window([](int){return 1023;}), -32768,-32768);
  check("  DiagStatus open bit", holdingRegs[DiagStatus], 1,1);
  check("short NTC (0)", window([](int){return 0;}), 32767,32767);
  check("  DiagStatus short bit", holdingRegs[DiagStatus], 2,2);
  check("intermittent open (2% at 1023)", window([](int i){return i%50==0?1023:511;}), -32768,-32768);
  check("0.5% spikes -> still a temperature", window([](int i){return i%200==0?1023:511;}), 2300,2600);
  check("recovers after fault", window([](int){return 511;}), 2490,2510);
  check("  DiagStatus cleared", holdingRegs[DiagStatus], 0,0);
  check("  AdcAverage = 511.0 counts", holdingRegs[AdcAverage], 5110,5110);
  check("  uptime = 9 s (9 windows so far)", holdingRegs[UptimeLo], 9,9);
  check("  boot count = 1 on erased EEPROM", holdingRegs[BootCount], 1,1);
  holdingRegs[SlaveIDReg]=40; checkModbusSlaveIdUpdate();
  check("  ID change saved, no save-fail bit", (holdingRegs[DiagStatus]&4)==0 && currentSlaveId==40, 1,1);
  setup(); check("  boot count = 2 after second setup()", holdingRegs[BootCount], 2,2);
  check("  slave ID 40 retained after 'reboot'", holdingRegs[SlaveIDReg], 40,40);
  check("slave ID still 40 at end", holdingRegs[SlaveIDReg], 40,40);
  // each TCNT1 read advances 80 cycles -> ISR body = 80 cycles = 5.0 us
  check("ISR time last second = 5.0 us", holdingRegs[IsrTimeMax], 50,50);
  TCNT1.step=160; window([](int){return 511;}); window([](int){return 511;});
  check("ISR time last second = 10.0 us", holdingRegs[IsrTimeMax], 100,100);
  TCNT1.step=40; window([](int){return 511;}); window([](int){return 511;});
  check("ISR time last second = 2.5 us", holdingRegs[IsrTimeMax], 25,25);
  check("ISR max since power-up stays 10.0 us", holdingRegs[IsrTimeMaxEver], 100,100);
  // Heartbeat LED: run loop-side updates every 5 ms for a span, windows every 1000 ms
  auto runFor = [](unsigned long ms, bool windows, uint16_t adc) {
    for (unsigned long t = 0; t < ms; t += 5) {
      fakeMillis += 5;
      if (windows && fakeMillis % 1000 == 0) { window([adc](int){return adc;}); }
      updateHeartbeat();
    }
  };
  runFor(1000, true, 511); ledToggles = 0;
  runFor(4000, true, 511);
  check("LED 1 Hz when OK (8 toggles in 4 s)", ledToggles, 7,9);
  runFor(1000, true, 1023); ledToggles = 0;
  runFor(4000, true, 1023);
  check("LED 4 Hz on NTC fault (32 toggles in 4 s)", ledToggles, 30,34);
  runFor(2000, false, 511); ledToggles = 0;
  runFor(3000, false, 511);
  check("LED steady when sampler stops (no toggles)", ledToggles, 0,0);
  check("LED off when sampler stops", ledState, 0,0);
  runFor(3000, true, 511);
  check("LED blinks again after sampler resumes", ledToggles, 4,7);
  printf("%d failures\n",fails); return fails;
}
