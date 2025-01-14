/****************************************************************************** 
 * simulation.cpp
 *
 * The main program that will be run. It will have the following functionality:
 *
 * - Send Data to Redis: Sends data to the redis database. Usually sending initial variables and data from STM32.
 * - Recieve Data from Redis: Recieves data from redis. Usually recieving data set from the tablet interface.
 * - Send Data to I2C: Sends data through the I2C connection to the STM32. Will be used to send PWM values and PTO status.
 * - Recieve Data from I2C: Recieves data from the STM32 peripheral. This data will send a variable and wether that variables should be increased or decreased.
 *
 ******************************************************************************/ 
#include <iostream>
#include <string>
#include <cstdint>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include "i2c_interface.h"
#include "redis_interface.h"

// variables determined from config file
static int MIN_AUGER_PIVOT_ANGLE;
static int MAX_AUGER_PIVOT_ANGLE;
static int MIN_AUGER_FOLD_ANGLE;
static int MAX_AUGER_FOLD_ANGLE;
static int MIN_SPOUT_TILT_ANGLE;
static int MAX_SPOUT_TILT_ANGLE;
static int MIN_SPOUT_ROTATION_ANGLE;
static int MAX_SPOUT_ROTATION_ANGLE;
static int MIN_GATE_OPEN_PERCENT;
static int MAX_GATE_OPEN_PERCENT;
static int AUGER_PIVOT_SPEED_REF;
static int AUGER_FOLD_SPEED_REF;
static int SPOUT_TILT_SPEED_REF;
static int SPOUT_ROTATION_SPEED_REF;
static int GATE_SPEED_REF;

// variables sent from the I2C connection from the STM32
static int augerPivotAngle = 90;
static int augerFoldAngle = 90;
static int spoutTiltAngle = 45;
static int spoutRotationAngle = 90;
static int gateOpenPercent = 0;
static int oldAugerPivotAngle = 0;
static int oldAugerFoldAngle = 0;
static int oldSpoutTiltAngle = 0;
static int oldSpoutRotationAngle = 0;
static int oldGateOpenPercent = 0;

// variables to be sent to the STM32 
static uint8_t augerPivotUpPWM = 0;
static uint8_t augerPivotDownPWM = 0;
static uint8_t augerFoldFoldPWM = 0;
static uint8_t augerFoldUnfoldPWM = 0;
static uint8_t spoutTiltUpPWM = 0;
static uint8_t spoutTiltDownPWM = 0;
static uint8_t spoutRotationCWPWM = 0;
static uint8_t spoutRotationCCWPWM = 0;
static uint8_t gateOpenPWM = 0;
static uint8_t gateClosePWM = 0;
static bool ptoOnOff = false;
static uint8_t oldAugerPivotUpPWM = 0;
static uint8_t oldAugerPivotDownPWM = 0;
static uint8_t oldAugerFoldFoldPWM = 0;
static uint8_t oldAugerFoldUnfoldPWM = 0;
static uint8_t oldSpoutTiltUpPWM = 0;
static uint8_t oldSpoutTiltDownPWM = 0;
static uint8_t oldSpoutRotationCWPWM = 0;
static uint8_t oldSpoutRotationCCWPWM = 0;
static uint8_t oldGateOpenPWM = 0;
static uint8_t oldGateClosePWM = 0;
static bool oldPtoOnOff = false;

// Function to load the default values from the config file
static bool loadConfigValues(const std::string &filename)
{
    // opens the config file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open config file: " << filename << std::endl;
        return false;
    }

    // use to while loop to read each line
    std::string line;
    while (std::getline(file, line)) {
        // move on if the line does not exist
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // splits the string at the "=" sign
        std::size_t delimiterPos = line.find('=');
        if (delimiterPos == std::string::npos) {
            continue; 
        }
        std::string key   = line.substr(0, delimiterPos);
        std::string value = line.substr(delimiterPos + 1);

        // trims the lines of whitespace
        auto trim = [](std::string &s) {
            while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
                s.erase(s.begin());
            }
            while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
                s.pop_back();
            }
        };
        trim(key);
        trim(value);

        int intValue = std::stoi(value);

        // fills the values taken from the config file into their specific variables
        if (key == "MIN_AUGER_PIVOT_ANGLE") {
            MIN_AUGER_PIVOT_ANGLE = intValue;
        } 
        else if (key == "MAX_AUGER_PIVOT_ANGLE") {
            MAX_AUGER_PIVOT_ANGLE = intValue;
        } 
        else if (key == "MIN_AUGER_FOLD_ANGLE") {
            MIN_AUGER_FOLD_ANGLE = intValue;
        } 
        else if (key == "MAX_AUGER_FOLD_ANGLE") {
            MAX_AUGER_FOLD_ANGLE = intValue;
        } 
        else if (key == "MIN_SPOUT_TILT_ANGLE") {
            MIN_SPOUT_TILT_ANGLE = intValue;
        } 
        else if (key == "MAX_SPOUT_TILT_ANGLE") {
            MAX_SPOUT_TILT_ANGLE = intValue;
        } 
        else if (key == "MIN_SPOUT_ROTATION_ANGLE") {
            MIN_SPOUT_ROTATION_ANGLE = intValue;
        } 
        else if (key == "MAX_SPOUT_ROTATION_ANGLE") {
            MAX_SPOUT_ROTATION_ANGLE = intValue;
        } 
        else if (key == "MIN_GATE_OPEN_PERCENT") {
            MIN_GATE_OPEN_PERCENT = intValue;
        } 
        else if (key == "MAX_GATE_OPEN_PERCENT") {
            MAX_GATE_OPEN_PERCENT = intValue;
        } 
        else if (key == "AUGER_PIVOT_SPEED_REF") {
            AUGER_PIVOT_SPEED_REF = intValue;
        } 
        else if (key == "AUGER_FOLD_SPEED_REF") {
            AUGER_FOLD_SPEED_REF = intValue;
        } 
        else if (key == "SPOUT_TILT_SPEED_REF") {
            SPOUT_TILT_SPEED_REF = intValue;
        } 
        else if (key == "SPOUT_ROTATION_SPEED_REF") {
            SPOUT_ROTATION_SPEED_REF = intValue;
        } 
        else if (key == "GATE_SPEED_REF") {
            GATE_SPEED_REF = intValue;
        }
    }

    file.close();
    return true;
}

// Function to update that variables within our code
static void updateVariable(int variableID, bool increment)
{
    switch (variableID) {
        case 12:
            if (increment) {
                if (augerPivotAngle < MAX_AUGER_PIVOT_ANGLE) {
                    augerPivotAngle += AUGER_PIVOT_SPEED_REF;
                }
            } else {
                if (augerPivotAngle > MIN_AUGER_PIVOT_ANGLE) {
                    augerPivotAngle -= AUGER_PIVOT_SPEED_REF;
                }
            }
            break;
        case 13:
            if (increment) {
                if (augerFoldAngle < MAX_AUGER_FOLD_ANGLE) {
                    augerFoldAngle += AUGER_FOLD_SPEED_REF;
                }
            } else {
                if (augerFoldAngle > MIN_AUGER_FOLD_ANGLE) {
                    augerFoldAngle -= AUGER_FOLD_SPEED_REF;
                }
            }
            break;
        case 14:
            if (increment) {
                if (spoutTiltAngle < MAX_SPOUT_TILT_ANGLE) {
                    spoutTiltAngle += SPOUT_TILT_SPEED_REF;
                }
            } else {
                if (spoutTiltAngle > MIN_SPOUT_TILT_ANGLE) {
                    spoutTiltAngle -= SPOUT_TILT_SPEED_REF;
                }
            }
            break;
        case 15:
            if (increment) {
                if (spoutRotationAngle < MAX_SPOUT_ROTATION_ANGLE) {
                    spoutRotationAngle += SPOUT_ROTATION_SPEED_REF;
                }
            } else {
                if (spoutRotationAngle > MIN_SPOUT_ROTATION_ANGLE) {
                    spoutRotationAngle -= SPOUT_ROTATION_SPEED_REF;
                }
            }
            break;
        case 16:
            if (increment) {
                if (gateOpenPercent < MAX_GATE_OPEN_PERCENT) {
                    gateOpenPercent += GATE_SPEED_REF;
                }
            } else {
                if (gateOpenPercent > MIN_GATE_OPEN_PERCENT) {
                    gateOpenPercent -= GATE_SPEED_REF;
                }
            }
            break;
        default:
            break;
    }
}

// Function to send inputted values into the redis server
static void sendUpdatedValueToRedis(const std::string &key, int value)
{
    int rc = sendDataToRedis("127.0.0.1", 6379, key, std::to_string(value), "");
    if (rc != 0) {
        std::cerr << "Failed to update Redis key: " << key << std::endl;
    } else {
        std::cout << "Updated Redis key: " << key << " => " << value << std::endl;
    }
}

int main()
{
    std::cout << "Starting simulation." << std::endl;

    // read the config file
    if (!loadConfigValues("config.txt")){
        std::cerr << "Config File does not exist. Exiting Program.\n";
        return 1;
    }

    // Open I2C connection
    int fileI2C = openI2C("/dev/i2c-1", 0x02);
    if (fileI2C < 0) {
        std::cerr << "Could not open I2C device. Exiting Program." << std::endl;
        return 1;
    }

    // Send initial Values to the Redis server
    sendUpdatedValueToRedis("auger_pivot_angle_min", MIN_AUGER_PIVOT_ANGLE);
    sendUpdatedValueToRedis("auger_pivot_angle_max", MAX_AUGER_PIVOT_ANGLE);
    sendUpdatedValueToRedis("auger_fold_angle_min", MIN_AUGER_FOLD_ANGLE);
    sendUpdatedValueToRedis("auger_fold_angle_max", MAX_AUGER_FOLD_ANGLE);
    sendUpdatedValueToRedis("spout_tilt_angle_min", MIN_SPOUT_TILT_ANGLE);
    sendUpdatedValueToRedis("spout_tilt_angle_max", MAX_SPOUT_TILT_ANGLE);
    sendUpdatedValueToRedis("spout_rotation_angle_min", MIN_SPOUT_ROTATION_ANGLE);
    sendUpdatedValueToRedis("spout_rotation_angle_max", MAX_SPOUT_ROTATION_ANGLE);
    sendUpdatedValueToRedis("gate_angle_min", MIN_GATE_OPEN_PERCENT);
    sendUpdatedValueToRedis("gate_angle_max", MAX_GATE_OPEN_PERCENT);
    sendUpdatedValueToRedis("auger_pivot_speed_ref", AUGER_PIVOT_SPEED_REF);
    sendUpdatedValueToRedis("auger_fold_speed_ref", AUGER_FOLD_SPEED_REF);
    sendUpdatedValueToRedis("spout_tilt_speed_ref", SPOUT_TILT_SPEED_REF);
    sendUpdatedValueToRedis("spout_rotation_speed_ref", SPOUT_ROTATION_SPEED_REF);
    sendUpdatedValueToRedis("gate_speed_ref", GATE_SPEED_REF);

    while (true) {
        uint8_t byteIn = 0;
        // read the data from the I2C connection
        int bytesRead = readFromI2C(fileI2C, &byteIn, 1);
        if (bytesRead == 1) {
            // Extact the variable ID and increment (Will have to modify this a bit)
            int  variableID = (byteIn & 0x7F);
            bool increment  = ((byteIn & 0x80) != 0);

            updateVariable(variableID, increment);

            // If the angles/percents have changed, push updates to Redis
            bool valueChanged = false;
            switch (variableID) {
                case 12:
                    if (augerPivotAngle != oldAugerPivotAngle) {
                        oldAugerPivotAngle = augerPivotAngle;
                        valueChanged = true;
                    }
                    break;
                case 13:
                    if (augerFoldAngle != oldAugerFoldAngle) {
                        oldAugerFoldAngle = augerFoldAngle;
                        valueChanged = true;
                    }
                    break;
                case 14:
                    if (spoutTiltAngle != oldSpoutTiltAngle) {
                        oldSpoutTiltAngle = spoutTiltAngle;
                        valueChanged = true;
                    }
                    break;
                case 15:
                    if (spoutRotationAngle != oldSpoutRotationAngle) {
                        oldSpoutRotationAngle = spoutRotationAngle;
                        valueChanged = true;
                    }
                    break;
                case 16:
                    if (gateOpenPercent != oldGateOpenPercent) {
                        oldGateOpenPercent = gateOpenPercent;
                        valueChanged = true;
                    }
                    break;
                default:
                    break;
            }

            // Send the value to redis if it has changed
            if (valueChanged) {
                switch (variableID) {
                    case 12:
                        sendUpdatedValueToRedis("auger_pivot_angle", augerPivotAngle);
                        break;
                    case 13:
                        sendUpdatedValueToRedis("auger_fold_angle", augerFoldAngle);
                        break;
                    case 14:
                        sendUpdatedValueToRedis("spout_tilt_angle", spoutTiltAngle);
                        break;
                    case 15:
                        sendUpdatedValueToRedis("spout_rotation_angle", spoutRotationAngle);
                        break;
                    case 16:
                        sendUpdatedValueToRedis("gate_angle", gateOpenPercent);
                        break;
                }
            }
        }

        // Check if the auger pivot PWM variables have changed or not. Send them to the STM32 if they have.
        if (augerPivotUpPWM != oldAugerPivotUpPWM) {
            writeToI2C(fileI2C, 1, static_cast<uint16_t>(augerPivotUpPWM));
            oldAugerPivotUpPWM = augerPivotUpPWM;
        }

        if (augerPivotDownPWM != oldAugerPivotDownPWM) {
            writeToI2C(fileI2C, 2, static_cast<uint16_t>(augerPivotDownPWM));
            oldAugerPivotDownPWM = augerPivotDownPWM;
        }

        if (augerFoldFoldPWM != oldAugerFoldFoldPWM) {
            writeToI2C(fileI2C, 3, static_cast<uint16_t>(augerFoldFoldPWM));
            oldAugerFoldFoldPWM = augerFoldFoldPWM;
        }

        if (augerFoldUnfoldPWM != oldAugerFoldUnfoldPWM) {
            writeToI2C(fileI2C, 4, static_cast<uint16_t>(augerFoldUnfoldPWM));
            oldAugerFoldUnfoldPWM = augerFoldUnfoldPWM;
        }

        if (spoutTiltUpPWM != oldSpoutTiltUpPWM) {
            writeToI2C(fileI2C, 5, static_cast<uint16_t>(spoutTiltUpPWM));
            oldSpoutTiltUpPWM = spoutTiltUpPWM;
        }

        if (spoutTiltDownPWM != oldSpoutTiltDownPWM) {
            writeToI2C(fileI2C, 6, static_cast<uint16_t>(spoutTiltDownPWM));
            oldSpoutTiltDownPWM = spoutTiltDownPWM;
        }

        if (spoutRotationCWPWM != oldSpoutRotationCWPWM) {
            writeToI2C(fileI2C, 7, static_cast<uint16_t>(spoutRotationCWPWM));
            oldSpoutRotationCWPWM = spoutRotationCWPWM;
        }

        if (spoutRotationCCWPWM != oldSpoutRotationCCWPWM) {
            writeToI2C(fileI2C, 8, static_cast<uint16_t>(spoutRotationCCWPWM));
            oldSpoutRotationCCWPWM = spoutRotationCCWPWM;
        }

        if (gateOpenPWM != oldGateOpenPWM) {
            writeToI2C(fileI2C, 9, static_cast<uint16_t>(gateOpenPWM));
            oldGateOpenPWM = gateOpenPWM;
        }

        if (gateClosePWM != oldGateClosePWM) {
            writeToI2C(fileI2C, 10, static_cast<uint16_t>(gateClosePWM));
            oldGateClosePWM = gateClosePWM;
        }

        if (ptoOnOff != oldPtoOnOff) {
            uint16_t ptoBit = ptoOnOff ? 1 : 0; 
            writeToI2C(fileI2C, 11, ptoBit);
            oldPtoOnOff = ptoOnOff;
        }

        usleep(100 * 1000); // Wait for 100 ms
    }

    closeI2C(fileI2C); //Close file
    return 0;
}
