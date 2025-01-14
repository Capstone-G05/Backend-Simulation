/****************************************************************************** 
 * i2c_interface.h
 *
 * The header file that will allow for the I2C functions will be used in the main program.
 *
 ******************************************************************************/ 
#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <string>
#include <cstdint>

int openI2C(const std::string &i2cBus, int address);
int readFromI2C(int file, uint8_t* buffer, size_t length);
bool writeToI2C(int file, uint8_t varID, uint16_t data);
void closeI2C(int file);

#endif
