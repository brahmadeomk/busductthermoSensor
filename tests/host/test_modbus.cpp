// Host test: Modbus RTU slave library with a simulated serial port and clock.
#include "SimpleModbusSlave.h"
#include <deque>
#include <vector>
#include <cstdio>
static unsigned long now_us = 0;
unsigned long micros(){ return now_us; } unsigned long millis(){ return now_us/1000; }
void delayMicroseconds(unsigned int u){ now_us += u; }
void pinMode(uint8_t,uint8_t){} void digitalWrite(uint8_t,uint8_t){}
std::deque<uint8_t> rx; std::vector<uint8_t> tx;
void HardwareSerial::begin(unsigned long, uint8_t){}
int HardwareSerial::available(){ return rx.size(); }
int HardwareSerial::read(){ int b=rx.front(); rx.pop_front(); return b; }
size_t HardwareSerial::write(const uint8_t*b,size_t n){ tx.insert(tx.end(),b,b+n); return n; }
void HardwareSerial::flush(){}
HardwareSerial Serial;
uint16_t crc(const std::vector<uint8_t>&f){ uint16_t c=0xFFFF; for(uint8_t b:f){c^=b; for(int j=0;j<8;j++) c=(c&1)?(c>>1)^0xA001:c>>1;} return c; }
std::vector<uint8_t> req(std::vector<uint8_t> f){ uint16_t c=crc(f); f.push_back(c&0xFF); f.push_back(c>>8); return f; }
unsigned int regs[22];
int fails=0;
std::vector<uint8_t> run(const std::vector<uint8_t>& f){
  tx.clear(); for(auto b:f) rx.push_back(b);
  modbus_update();              // drains, frame not complete yet
  now_us += 4000; modbus_update(); // silence -> process
  return tx;
}
void check(const char*name,bool ok){ printf("%s: %s\n",name,ok?"PASS":"FAIL"); if(!ok) fails++; }
bool validCrc(const std::vector<uint8_t>&r){ if(r.size()<3) return false; std::vector<uint8_t> b(r.begin(),r.end()-2); uint16_t c=crc(b); return r[r.size()-2]==(c&0xFF)&&r.back()==(c>>8); }
int main(){
  for(int i=0;i<22;i++) regs[i]=1000+i; regs[3]=(unsigned int)(int16_t)-1234;
  modbus_configure(&Serial,9600,6,71,3,22,regs);
  auto r=run(req({71,3,0,0,0,22}));
  check("FC3 read all 22", r.size()==49 && r[0]==71 && r[2]==44 && validCrc(r) && r[3]==(1000>>8) && r[4]==(1000&0xFF) && r[9]==0xFB && r[10]==0x2E);
  r=run(req({72,3,0,0,0,1})); check("other slave ignored", r.empty());
  auto bad=req({71,3,0,0,0,1}); bad[6]^=1; r=run(bad); check("bad CRC ignored", r.empty());
  r=run(req({71,3,0,20,0,5})); check("FC3 out of range -> exc 2", r.size()==5 && r[1]==0x83 && r[2]==2 && validCrc(r));
  r=run(req({71,3,0,10,0xFF,0xFF})); check("FC3 count wrap -> exc 3", r.size()==5 && r[2]==3);
  r=run(req({71,3,0,0,0,0})); check("FC3 count 0 -> exc 3", r.size()==5 && r[2]==3);
  r=run(req({71,6,0,0,0,5})); check("FC6 to measurement reg -> exc 2", r.size()==5 && r[1]==0x86 && r[2]==2 && regs[0]==1000);
  auto w=req({71,6,0,21,0,80}); r=run(w); check("FC6 slave ID echo", r==w && regs[21]==80);
  regs[21]=1021;
  r=run(req({0,6,0,21,0,90})); check("broadcast write ignored", r.empty() && regs[21]==1021);
  r=run(req({71,16,0,21,0,1,2,0,77})); check("FC16 slave ID", r.size()==8 && validCrc(r) && regs[21]==77 && r[5]==1);
  r=run(req({71,4,0,0,0,1})); check("FC4 -> exc 1", r.size()==5 && r[1]==0x84 && r[2]==1);
  // two frames merged with no silence: should not produce a bogus reply
  tx.clear(); for(auto b:req({72,3,0,0,0,1})) rx.push_back(b); for(auto b:req({71,3,0,0,0,1})) rx.push_back(b);
  modbus_update(); now_us+=4000; modbus_update(); check("merged frames -> no reply (needs fast loop)", tx.empty());
  // frame with intra-frame gap < 3.5 char is still assembled
  tx.clear(); auto f=req({71,3,0,0,0,1}); for(int i=0;i<4;i++) rx.push_back(f[i]); modbus_update(); now_us+=2500;
  for(int i=4;i<8;i++) rx.push_back(f[i]); modbus_update(); now_us+=4000; modbus_update();
  check("2.5 ms gap inside frame tolerated", tx.size()==7 && validCrc(tx));
  // overflow
  tx.clear(); for(int i=0;i<70;i++) rx.push_back(71); modbus_update(); now_us+=4000; modbus_update();
  check("overflow -> no reply, no crash", tx.empty());
  r=run(req({71,3,0,0,0,1})); check("recovers after overflow", r.size()==7);
  // map with registers appended after the writable slave ID register
  unsigned int regs24[24] = {0};
  modbus_configure(&Serial,9600,6,71,3,24,regs24);
  modbus_set_writable_register(21);
  r=run(req({71,3,0,22,0,2})); check("appended regs 22-23 readable", r.size()==9 && validCrc(r));
  r=run(req({71,6,0,21,0,55})); check("write slave ID at 21 with appended regs", r.size()==8 && regs24[21]==55);
  r=run(req({71,6,0,23,0,1})); check("write to last (23) rejected", r.size()==5 && r[2]==2 && regs24[23]==0);
  {
    unsigned int regs32[32]; for(int i=0;i<32;i++) regs32[i]=i;
    modbus_configure(&Serial,9600,6,71,3,32,regs32);
    r=run(req({71,3,0,0,0,32})); check("read all 32 regs in one request", r.size()==69 && validCrc(r) && r[3+2*31+1]==31);
    run(req({72,3,0,0,0,1}));                       // other slave
    auto b2=req({71,3,0,0,0,1}); b2[7]^=1; run(b2); // CRC error
    run(req({71,4,0,0,0,1}));                       // exception
    run(req({71,6,0,31,0,1,0}));                    // malformed FC6 (9 bytes, CRC still valid)
    const ModbusStats &st = modbus_stats();
    printf("  stats: bus=%u req=%u crc=%u frame=%u exc=%u\n", st.busFrames, st.requests, st.crcErrors, st.frameErrors, st.exceptions);
    check("stats counters", st.busFrames==5 && st.requests==3 && st.crcErrors==1 && st.frameErrors==1 && st.exceptions==1);
  }
  printf("%d failures\n",fails); return fails;
}
