#pragma once
#include <stdint.h>
struct EEPROMClass { uint8_t read(int); void update(int, uint8_t); }; extern EEPROMClass EEPROM;
