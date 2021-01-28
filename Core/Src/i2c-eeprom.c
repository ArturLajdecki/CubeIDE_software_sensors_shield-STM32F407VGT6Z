/** Put this in the src folder **/

#include <stdio.h>
#include <stdlib.h>
#include "i2c-eeprom.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_EEPROM 0xA0 // change this according to your setup




// MemAddress - Address in memory where data is going to be send
// data - Data to write
// size - Amount of data to be sent
char data_eeprom[255];

void eeprom_write(uint16_t position, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, SLAVE_ADDRESS_EEPROM, position, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
	HAL_Delay(5);
}

uint8_t eeprom_read (uint16_t position)
{
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, SLAVE_ADDRESS_EEPROM, position, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
	return data;
}

void eeprom_write_string(uint16_t position, char *data)
{
	eeprom_write(position, strlen(data) );

	for(int i = 0; i < strlen(data); i++)
	{
		eeprom_write(position + 1 + i, data[i]);
	}
}

void eeprom_read_string(uint16_t position)
{
	uint16_t string = eeprom_read(position);

	memset(data_eeprom, 0, 255);

	for(int i = 0; i < string; i++)
	{
		data_eeprom[i] = read(position + 1 + i);
	}
}

