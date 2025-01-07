/****************************************************************************** 
 * simulation.cpp
 *
 * This program views an I2C bus and detemines what variable type was sent as well
 * as wether that variable should be incremented or decremented.
 *
 * The message sent will be one of the five variables:
 *   1) Auger pivot angle
 *   2) Auger fold angle
 *   3) Spout tilt angle
 *   4) Spout rotation angle
 *   5) Gate open percentage
 *
 * After that message has been received it is checked to make sure 
 * that it is within proper bounds and then sent to the Redis API/database.
 ******************************************************************************/ 

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "send_data.h"

// Bounds for data to be sent via Redis
static const int MIN_AUGER_PIVOT_ANGLE    = 0;
static const int MAX_AUGER_PIVOT_ANGLE    = 180;

static const int MIN_AUGER_FOLD_ANGLE     = 0;
static const int MAX_AUGER_FOLD_ANGLE     = 180;

static const int MIN_SPOUT_TILT_ANGLE     = 0;
static const int MAX_SPOUT_TILT_ANGLE     = 90;

static const int MIN_SPOUT_ROTATION_ANGLE = 0;
static const int MAX_SPOUT_ROTATION_ANGLE = 180;

static const int MIN_GATE_OPEN_PERCENT    = 0;
static const int MAX_GATE_OPEN_PERCENT    = 100;

// The initial values of the variables
static int augerPivotAngle    = 90;
static int augerFoldAngle     = 90;
static int spoutTiltAngle     = 45;
static int spoutRotationAngle = 90;
static int gateOpenPercent    = 0;

// Old values to compare to see if anything has changed
static int oldAugerPivotAngle    = 0;
static int oldAugerFoldAngle     = 0;
static int oldSpoutTiltAngle     = 0;
static int oldSpoutRotationAngle = 0;
static int oldGateOpenPercent    = 0;

// Reads the data that comes from the I2C bus
int readFromI2C(int file, uint8_t* buffer, size_t length) {
    ssize_t bytesRead = ::read(file, buffer, length);
    if (bytesRead < 0 || (static_cast<size_t>(bytesRead) < length)) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
    }
    return static_cast<int>(bytesRead);
}

// Function to update the variable that was sent via the I2C connection
void updateVariable(int variableID, bool increment) {
    switch (variableID) {
        case 0: { 
            if (increment) {
                if (augerPivotAngle < MAX_AUGER_PIVOT_ANGLE) {
                    augerPivotAngle++;
                }
            } else {
                if (augerPivotAngle > MIN_AUGER_PIVOT_ANGLE) {
                    augerPivotAngle--;
                }
            }
        } break;
        case 1: {
            if (increment) {
                if (augerFoldAngle < MAX_AUGER_FOLD_ANGLE) {
                    augerFoldAngle++;
                }
            } else {
                if (augerFoldAngle > MIN_AUGER_FOLD_ANGLE) {
                    augerFoldAngle--;
                }
            }
        } break;
        case 2: {
            if (increment) {
                if (spoutTiltAngle < MAX_SPOUT_TILT_ANGLE) {
                    spoutTiltAngle++;
                }
            } else {
                if (spoutTiltAngle > MIN_SPOUT_TILT_ANGLE) {
                    spoutTiltAngle--;
                }
            }
        } break;
        case 3: {
            if (increment) {
                if (spoutRotationAngle < MAX_SPOUT_ROTATION_ANGLE) {
                    spoutRotationAngle++;
                }
            } else {
                if (spoutRotationAngle > MIN_SPOUT_ROTATION_ANGLE) {
                    spoutRotationAngle--;
                }
            }
        } break;
        case 4: {
            if (increment) {
                if (gateOpenPercent < MAX_GATE_OPEN_PERCENT) {
                    gateOpenPercent++;
                }
            } else {
                if (gateOpenPercent > MIN_GATE_OPEN_PERCENT) {
                    gateOpenPercent--;
                }
            }
        } break;
        default:
            std::cerr << "Unknown variableID: " << variableID << std::endl;
            break;
    }
}

// Function that sends the updated value to the redis server
void sendUpdatedValueToRedis(const std::string &key, int value) {
    // Assume Redis runs on localhost at port 6379 with no password
    int result = sendDataToRedis("127.0.0.1", 6379, key, std::to_string(value), "");
    if (result != 0) {
        std::cerr << "[ERROR] Failed to update Redis key: " << key << std::endl;
    } else {
        std::cout << "[INFO] Updated Redis key: " << key << " => " << value << std::endl;
    }
}

// Main function
int main() {
    std::cout << "Starting simulation." << std::endl;

    // I2C bus and address setup
    const std::string i2cBus = "/dev/i2c-1";
    int address = 0x10;

    // Open the I2C bus
    int fileI2C = open(i2cBus.c_str(), O_RDWR | O_NONBLOCK);
    if (fileI2C < 0) {
        std::cerr << "Failed to open the I2C bus: " 
                  << i2cBus << " (" << std::strerror(errno) << ")" << std::endl;
        return 1;
    }

    // Set the I2C slave address
    if (ioctl(fileI2C, I2C_SLAVE, address) < 0) {
        std::cerr << "Failed to set I2C address to 0x" << std::hex 
                  << address << " (" << std::strerror(errno) << ")" << std::endl;
        close(fileI2C);
        return 1;
    }

    // Main loop
    while (true) {
        // Attempt to read two bytes from the I2C connection
        uint8_t buffer[2];
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = readFromI2C(fileI2C, buffer, sizeof(buffer));
        // If the correct amount of bytes are read
        if (bytesRead == 2) {
            // Extract the data
            int  variableID = buffer[0];
            bool increment  = (buffer[1] == 1);

            // Update the variable
            updateVariable(variableID, increment);

            // Determine if the new value differs from the old
            bool valueChanged = false;
            switch (variableID) {
                case 0:
                    if (augerPivotAngle != oldAugerPivotAngle) {
                        oldAugerPivotAngle = augerPivotAngle;
                        valueChanged = true;
                    }
                    break;
                case 1:
                    if (augerFoldAngle != oldAugerFoldAngle) {
                        oldAugerFoldAngle = augerFoldAngle;
                        valueChanged = true;
                    }
                    break;
                case 2:
                    if (spoutTiltAngle != oldSpoutTiltAngle) {
                        oldSpoutTiltAngle = spoutTiltAngle;
                        valueChanged = true;
                    }
                    break;
                case 3:
                    if (spoutRotationAngle != oldSpoutRotationAngle) {
                        oldSpoutRotationAngle = spoutRotationAngle;
                        valueChanged = true;
                    }
                    break;
                case 4:
                    if (gateOpenPercent != oldGateOpenPercent) {
                        oldGateOpenPercent = gateOpenPercent;
                        valueChanged = true;
                    }
                    break;
                default:
                    std::cerr << "Unknown variableID: " << variableID << std::endl;
                    break;
            }

            // If the value changed, send it to Redis
            if (valueChanged) {
                switch (variableID) {
                    case 0:
                        sendUpdatedValueToRedis("auger_pivot_angle", augerPivotAngle);
                        break;
                    case 1:
                        sendUpdatedValueToRedis("auger_fold_angle", augerFoldAngle);
                        break;
                    case 2:
                        sendUpdatedValueToRedis("spout_tilt_angle", spoutTiltAngle);
                        break;
                    case 3:
                        sendUpdatedValueToRedis("spout_rotation_angle", spoutRotationAngle);
                        break;
                    case 4:
                        sendUpdatedValueToRedis("gate_angle", gateOpenPercent);
                        break;
                }
            }
        }

        usleep(50 * 1000); // Sleep for 50 ms
    }

    close(fileI2C);
    std::cout << "Program completed." << std::endl;
    return 0;
}
