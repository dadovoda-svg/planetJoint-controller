#pragma once
#include <stdint.h>
#include <stddef.h>
#define SERIAL_8N1 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline void delayMicroseconds(unsigned int) {}
class HardwareSerial { public: void begin(unsigned long, unsigned long=0, int=-1, int=-1){} int available(){return 0;} int read(){return -1;} size_t write(const uint8_t*, size_t n){return n;} void flush(){} void setPins(int=-1,int=-1,int=-1,int=-1){} void setHwFlowCtrlMode(int){} void setMode(int){} };
