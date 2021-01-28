#include "stm32f4xx_hal.h"


void eeprom_write(uint16_t position, uint8_t data);

uint8_t eeprom_read (uint16_t position);

void eeprom_write_string(uint16_t position, char *data);

void eeprom_read_string(uint16_t position);

void eeprom_write_string(uint16_t position, char *data);

void eeprom_read_string(uint16_t position);
