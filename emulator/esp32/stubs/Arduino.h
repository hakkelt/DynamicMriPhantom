// Minimal Arduino-ESP32 API for the host ESP32-S3 emulator (emulator/esp32/sim.cpp).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define IRAM_ATTR
#define F_CPU 240000000UL
#define LOW 0
#define HIGH 1
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define RISING 0x01

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))

class HostSerial {
public:
    void begin(unsigned long baud);
    int available();
    int read();
    size_t print(const __FlashStringHelper* s) { return print(reinterpret_cast<const char*>(s)); }
    size_t print(const char* s);
    size_t print(char c);
    size_t print(int v) { return print((long)v); }
    size_t print(unsigned int v) { return print((unsigned long)v); }
    size_t print(long v);
    size_t print(unsigned long v);
    template <typename T>
    size_t println(T v) { size_t n = print(v); return n + print('\n'); }
    size_t println() { return print('\n'); }
};
extern HostSerial Serial;

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t level);
int digitalRead(uint8_t pin);
void attachInterrupt(uint8_t pin, void (*isr)(void), int mode);
#define digitalPinToInterrupt(p) (p)

typedef struct { int unused; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
void sim_enter_critical();
void sim_exit_critical();
#define portENTER_CRITICAL(m) ((void)(m), sim_enter_critical())
#define portEXIT_CRITICAL(m) ((void)(m), sim_exit_critical())
#define portENTER_CRITICAL_ISR(m) ((void)(m), sim_enter_critical())
#define portEXIT_CRITICAL_ISR(m) ((void)(m), sim_exit_critical())
