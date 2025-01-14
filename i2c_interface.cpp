/****************************************************************************** 
 * i2c_interface.cpp
 *
 * The functions that are used to interface with I2C
 *
 * - openI2C(i2cBus,address): Opens the I2C connection.
 * - readFromI2C(int,buffer,length): reads data from the I2C connection(i.e. reads data from STM32).
 * - writeToI2C(file,varID,data): writes data to the I2C connection(i.e. writes data to the STM32).
 * - closeI2C(file): closes the I2C connection.
 *
 ******************************************************************************/ 
#include "i2c_interface.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <linux/i2c-dev.h>

// Function to open a non blocking I2C connection
int openI2C(const std::string &i2cBus, int address) 
{
    // open the non clocking I2C connection
    int flags = O_NONBLOCK;
    int file = open(i2cBus.c_str(), flags);
    if (file < 0) {
        return -1;
    }

    // sets up the connection to the STM32 device
    if (ioctl(file, I2C_SLAVE, address) < 0) {
        close(file);
        return -1;
    }

    return file;
}

// Function to read data from the opened I2C connection
int readFromI2C(int file, uint8_t* buffer, size_t length) 
{
    // read the bytes from the I2C connection
    ssize_t bytesRead = ::read(file, buffer, length);
    if (bytesRead < 0 || (static_cast<size_t>(bytesRead) < length)) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }
    // cast the recieved data and return it
    return static_cast<int>(bytesRead);
}

// function to write data to the I2C connected peripheral
bool writeToI2C(int file, uint8_t varID, uint16_t data) 
{
    // send the requested data in 3 parts(varID, byte0Data, byte1Data)
    uint8_t buffer[3];
    buffer[0] = varID;
    buffer[1] = static_cast<uint8_t>(data & 0xFF);
    buffer[2] = static_cast<uint8_t>((data >> 8) & 0xFF);

    // write the data to the connection and return boolean based on success
    ssize_t bytesWritten = ::write(file, buffer, 3);
    if (bytesWritten < 0) {
        return false;
    } else if (bytesWritten < 3) {
        return false;
    }
    return true;
}

// function to close the I2C connection
void closeI2C(int file) 
{
    close(file);
}
