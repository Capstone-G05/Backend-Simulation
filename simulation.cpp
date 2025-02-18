/****************************************************************************** 
 * simulation.cpp
 *
 *
 *
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
 #include <cstdlib>
 #include <csignal>
 #include <fcntl.h>
 #include <sys/ioctl.h>
 #include <linux/i2c-dev.h>
 #include <hiredis/hiredis.h>
 
 // I2C bus and device addresses
 #define I2C_BUS "/dev/i2c-1"
 #define STM32_ADDRESS 0x10
 #define DAC_ADDRESS    0x60
 
 // Buffer sizes and delay
 #define STM_BUFFER_SIZE 4
 #define DAC_TX_BUFFER_SIZE 3
 #define DAC_RX_BUFFER_SIZE 2
 #define MESSAGE_DELAY_US 10000
 
 const char* redis_host = getenv("REDIS_HOST");
 const int redis_port = atoi(getenv("REDIS_PORT"));
 
 // DAC command masks
 const uint8_t DAC_WRITE_CMD_MASK = 0x00;  
 const uint8_t DAC_READ_CMD_MASK  = 0x06;  
 
 // Global file descriptors for I2C devices
 int fd_stm32 = -1;
 int fd_dac = -1;
 
 // Global variables for control values
 static uint16_t augerBottomPivotAngle = 90;
 static uint16_t augerTopAngle = 90;
 static uint16_t spoutTiltAngle = 45;
 static uint16_t headRotationAngle = 90;
 static uint16_t gateAngle = 0;
 static uint16_t ptoOnOff = 0;
 static uint16_t cropFillRate = 0;
 
 static uint16_t augerBottomPivotAngleMin = 0;
 static uint16_t augerBottomPivotAngleMax = 180;
 static uint16_t augerBottomPivotSpeedRef = 50;
 
 static uint16_t augerTopAngleMin = 0;
 static uint16_t augerTopAngleMax = 180;
 static uint16_t augerTopRefSpeed = 50;
 
 static uint16_t spoutTiltAngleMin = 0;
 static uint16_t spoutTiltAngleMax = 90;
 static uint16_t spoutTiltRefSpeed = 30;
 
 static uint16_t headRotationAngleMin = 0;
 static uint16_t headRotationAngleMax = 180;
 static uint16_t headRotationRefSpeed = 40;
 
 static uint16_t gateAngleMin = 0;
 static uint16_t gateAngleMax = 100;
 static uint16_t gateRefSpeed = 20;
 
 static uint16_t machineType = 0;
 static uint16_t frontWeight = 0;
 static uint16_t rearWeight = 0;
 
 int sendDataToRedis(const std::string &hostname,
     int port,
     const std::string &key,
     const std::string &value,
     const std::string &password);
 int getDataFromRedis(const std::string &hostname,
     int port,
     const std::string &key,
     std::string &outValue,
     const std::string &password);
 void sendUpdatedValueToRedis(const std::string &key, int value);
 int openI2CDevice(const std::string &i2cBus, int address);
 int readFromSTM32(int file, unsigned char *buffer, size_t length);
 bool writeToSTM32(int file, unsigned char *bytes, size_t size);
 int openDACDevice(const std::string &i2cBus, int address);
 int writeDAC(int dac_fd, uint8_t reg, uint16_t value);
 int readDAC(int dac_fd, uint8_t reg, uint16_t *value);
 void cleanup();
 
 void cleanup() {
     if (fd_stm32 >= 0) {
         close(fd_stm32);
     }
     if (fd_dac >= 0) {
         close(fd_dac);
     }
 }
 
 int openI2CDevice(const std::string &i2cBus, int address) {
     int file = open(i2cBus.c_str(), O_RDWR);
     if (file < 0) {
         perror("Failed to open I2C bus");
         return -1;
     }
     if (ioctl(file, I2C_SLAVE, address) < 0) {
         perror("Failed to set I2C slave address");
         close(file);
         return -1;
     }
     return file;
 }
 
 int readFromSTM32(int file, unsigned char *buffer, size_t length) {
     if (read(file, buffer, length) != (ssize_t)length) {
         perror("Failed to read bytes from STM32");
         return -1;
     }
 
     uint8_t header = buffer[0];
     uint8_t rw = header >> 7;
     uint8_t type = header & 0x7F;
     uint8_t index = buffer[1];
     uint16_t data = (buffer[2] << 8) | buffer[3];
     return 0;
 }
 
 bool writeToSTM32(int file, unsigned char *bytes, size_t size) {
     if (write(file, bytes, size) != (ssize_t)size) {
         perror("Failed to write bytes to STM32");
         return false;
     }
     std::cout << "Sent message to STM32: ";
     for (size_t i = 0; i < size; i++) {
         printf("%02X ", bytes[i]);
     }
     std::cout << std::endl;
     return true;
 }
 
 void buildI2CMessage(unsigned char *buffer, bool rw, uint8_t type, uint8_t index, uint16_t data) {
     buffer[0] = ((rw ? 1 : 0) << 7) | (type & 0x7F);
     buffer[1] = index;
     buffer[2] = (data >> 8) & 0xFF;
     buffer[3] = data & 0xFF;
 }
 
 int openDACDevice(const std::string &i2cBus, int address) {
     int file = open(i2cBus.c_str(), O_RDWR);
     if (file < 0) {
         perror("Failed to open I2C bus for DAC");
         return -1;
     }
     if (ioctl(file, I2C_SLAVE, address) < 0) {
         perror("Failed to set DAC I2C slave address");
         close(file);
         return -1;
     }
     return file;
 }
 
 int writeDAC(int dac_fd, uint8_t reg, uint16_t value) {
     if (reg > 0x1F) {
         std::cerr << "Invalid DAC register index. Must be between 0 and 31." << std::endl;
         return -1;
     }
     unsigned char tx_buffer[DAC_TX_BUFFER_SIZE];
     tx_buffer[0] = ((reg & 0x1F) << 3) | DAC_WRITE_CMD_MASK;
     tx_buffer[1] = (value >> 8) & 0xFF;
     tx_buffer[2] = value & 0xFF;
     if (write(dac_fd, tx_buffer, DAC_TX_BUFFER_SIZE) != DAC_TX_BUFFER_SIZE) {
         perror("Failed to write bytes to DAC");
         return -1;
     }
     std::cout << "DAC write: Reg " << static_cast<int>(reg)
             << " Value " << value << std::endl;
     return 0;
 }
 
 int readDAC(int dac_fd, uint8_t reg, uint16_t *value) {
     if (reg > 0x1F) {
         std::cerr << "Invalid DAC register index. Must be between 0 and 31." << std::endl;
         return -1;
     }
     unsigned char tx_buffer[1];
     unsigned char rx_buffer[DAC_RX_BUFFER_SIZE];
     tx_buffer[0] = ((reg & 0x1F) << 3) | DAC_READ_CMD_MASK;
     if (write(dac_fd, tx_buffer, 1) != 1) {
         perror("Failed to send read request to DAC");
         return -1;
     }
     usleep(MESSAGE_DELAY_US);
     if (read(dac_fd, rx_buffer, DAC_RX_BUFFER_SIZE) != DAC_RX_BUFFER_SIZE) {
         perror("Failed to read bytes from DAC");
         return -1;
     }
     *value = (rx_buffer[0] << 8) | rx_buffer[1];
     std::cout << "DAC read: Reg " << static_cast<int>(reg)
             << " Value " << *value << std::endl;
     return 0;
 }
 
 // Function to send data to the Redis server
 int sendDataToRedis(const std::string &hostname,
                     int port,
                     const std::string &key,
                     const std::string &value,
                     const std::string &password)
 {
     // Connect to Redis
     redisContext *c = redisConnect(hostname.c_str(), port);
     if (c == nullptr || c->err) {
         if (c) {
             std::cerr << "[ERROR] Connection error: " << c->errstr << std::endl;
             redisFree(c);
         } else {
             std::cerr << "[ERROR] Connection error: can't allocate redis context." << std::endl;
         }
         return -1;
     }
 
     // Set the password if needed
     if (!password.empty()) {
         redisReply *authReply = (redisReply *)redisCommand(c, "AUTH %s", password.c_str());
         if (!authReply || authReply->type == REDIS_REPLY_ERROR) {
             std::cerr << "[ERROR] Redis AUTH failed. Check password or Redis config.";
             if (authReply) {
                 std::cerr << " Error: " << authReply->str;
                 freeReplyObject(authReply);
             }
             std::cerr << std::endl;
             redisFree(c);
             return -1;
         }
         freeReplyObject(authReply);
     }
 
     // Send the SET command to Redis
     redisReply *reply = (redisReply *)redisCommand(c, "SET %s %s", key.c_str(), value.c_str());
     if (!reply) {
         std::cerr << "ERROR: " << c->errstr << std::endl;
         redisFree(c);
         return -1;
     }
 
     // Log the result if successful
     if (reply->type == REDIS_REPLY_STATUS) {
         std::cout << "Redis SET: " << key << " => " << value
                 << " (" << reply->str << ")" << std::endl;
     }
 
     freeReplyObject(reply);
     redisFree(c);
     return 0;
 }
 
 // Function to obtain data from the Redis server
 int getDataFromRedis(const std::string &hostname,
                     int port,
                     const std::string &key,
                     std::string &outValue,
                     const std::string &password)
 {
     // Connect to Redis
     redisContext *c = redisConnect(hostname.c_str(), port);
     if (c == nullptr || c->err) {
         if (c) {
             std::cerr << "[ERROR] Connection error: " << c->errstr << std::endl;
             redisFree(c);
         } else {
             std::cerr << "[ERROR] Connection error: can't allocate redis context." << std::endl;
         }
         return -1;
     }
 
     // Use password if applicable
     if (!password.empty()) {
         redisReply *authReply = (redisReply *)redisCommand(c, "AUTH %s", password.c_str());
         if (!authReply || authReply->type == REDIS_REPLY_ERROR) {
             std::cerr << "[ERROR] Redis AUTH failed.\n";
             if (authReply) {
                 std::cerr << "       Error: " << authReply->str << std::endl;
                 freeReplyObject(authReply);
             }
             redisFree(c);
             return -1;
         }
         freeReplyObject(authReply);
     }
 
     // Send the GET command to Redis
     redisReply *reply = (redisReply *)redisCommand(c, "GET %s", key.c_str());
     if (!reply) {
         std::cerr << "[ERROR] Command error: " << c->errstr << std::endl;
         redisFree(c);
         return -1;
     }
 
     if (reply->type == REDIS_REPLY_STRING) {
         outValue = reply->str;
     } else if (reply->type == REDIS_REPLY_NIL) {
         outValue.clear();
     } else {
         outValue.clear();
     }
 
     freeReplyObject(reply);
     redisFree(c);
     return 0;
 }
 
 void sendUpdatedValueToRedis(const std::string &key, int value) {
     int rc = sendDataToRedis(redis_host, redis_port, key, std::to_string(value), "");
     if (rc != 0) {
         std::cerr << "Failed to update Redis key: " << key << std::endl;
     } else {
         std::cout << "Updated Redis key: " << key << " => " << value << std::endl;
     }
 }
 
 void sendPWMValuesToRedis(const std::string &key, uint8_t address){
     unsigned char stm_tx_buffer[STM_BUFFER_SIZE] = {0};
     unsigned char stm_rx_buffer[STM_BUFFER_SIZE] = {0};
 
     buildI2CMessage(stm_tx_buffer, true, 0x02, address, 0x0000);
     writeToSTM32(fd_stm32, stm_tx_buffer, STM_BUFFER_SIZE);
     usleep(MESSAGE_DELAY_US);
     readFromSTM32(fd_stm32, stm_rx_buffer, STM_BUFFER_SIZE);
     uint8_t header = stm_rx_buffer[0];
     uint8_t rw = header >> 7; 
     uint8_t type = header & 0x7F; 
     uint8_t index = stm_rx_buffer[1];
     uint16_t data = (stm_rx_buffer[2] << 8) | stm_rx_buffer[3];
     sendUpdatedValueToRedis(key, data);
 }
 
 int main() {
     std::cout << "Starting Raspberry Pi Simulation." << std::endl;
     
     // Open I2C connections to STM32 and DAC
     fd_stm32 = openI2CDevice(I2C_BUS, STM32_ADDRESS);
     if (fd_stm32 < 0) {
         std::cerr << "Could not open I2C device for STM32. Exiting." << std::endl;
         return 1;
     }
     
     fd_dac = openDACDevice(I2C_BUS, DAC_ADDRESS);
     if (fd_dac < 0) {
         std::cerr << "Could not open I2C device for DAC. Exiting." << std::endl;
         cleanup();
         return 1;
     }
     
     // Obtain the initial values from Redis
     std::string redisValue;
     int rc;
     rc = getDataFromRedis(redis_host, redis_port, "auger_bottom_pivot_angle_max", redisValue, "");
     if (rc == 0 && !redisValue.empty()) augerBottomPivotAngleMax = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "auger_bottom_pivot_angle_min", redisValue, "");
     if (rc == 0 && !redisValue.empty()) augerBottomPivotAngleMin = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "auger_bottom_pivot_speed_ref", redisValue, "");
     if (rc == 0 && !redisValue.empty()) augerBottomPivotSpeedRef = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "auger_top_angle_min", redisValue, "");
     if (rc == 0 && !redisValue.empty()) augerTopAngleMin = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "auger_top_angle_max", redisValue, "");
     if (rc == 0 && !redisValue.empty()) augerTopAngleMax = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "auger_top_speed_ref", redisValue, "");
     if (rc == 0 && !redisValue.empty()) augerTopRefSpeed = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "spout_tilt_angle_min", redisValue, "");
     if (rc == 0 && !redisValue.empty()) spoutTiltAngleMin = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "spout_tilt_angle_max", redisValue, "");
     if (rc == 0 && !redisValue.empty()) spoutTiltAngleMax = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "spout_tilt_speed_ref", redisValue, "");
     if (rc == 0 && !redisValue.empty()) spoutTiltRefSpeed = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "head_rotation_angle_min", redisValue, "");
     if (rc == 0 && !redisValue.empty()) headRotationAngleMin = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "head_rotation_angle_max", redisValue, "");
     if (rc == 0 && !redisValue.empty()) headRotationAngleMax = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "head_rotation_speed_ref", redisValue, "");
     if (rc == 0 && !redisValue.empty()) headRotationRefSpeed = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "gate_angle_min", redisValue, "");
     if (rc == 0 && !redisValue.empty()) gateAngleMin = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "gate_angle_max", redisValue, "");
     if (rc == 0 && !redisValue.empty()) gateAngleMax = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "gate_speed_ref", redisValue, "");
     if (rc == 0 && !redisValue.empty()) gateRefSpeed = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "machine_type", redisValue, "");
     if (rc == 0 && !redisValue.empty()) machineType = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "front_weight", redisValue, "");
     if (rc == 0 && !redisValue.empty()) frontWeight = std::atoi(redisValue.c_str());
     
     rc = getDataFromRedis(redis_host, redis_port, "rear_weight", redisValue, "");
     if (rc == 0 && !redisValue.empty()) rearWeight = std::atoi(redisValue.c_str());
     
     int redisCheckCounter = 0;
     while (true) {
         // Obtain PWM values and send them to redis
         sendPWMValuesToRedis("auger_bottom_pivot_up_pwm", 0x08);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("auger_bottom_pivot_down_pwm", 0x04);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("auger_top_unfold_pwm", 0x0B);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("auger_top_fold_pwm", 0x0A);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("spout_tilt_up_pwm", 0x01);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("spout_tilt_down_pwm", 0x03);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("head_rotation_cw_pwm", 0x06);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("head_rotation_ccw_pwm", 0x07);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("gate_open_pwm", 0x05);
         usleep(MESSAGE_DELAY_US);
         sendPWMValuesToRedis("gate_close_pwm", 0x09);
         usleep(MESSAGE_DELAY_US);
         // Tandem Float and Tandem Cutoff not done due to no redis values being made for them
 
         // Check heartbeat
         unsigned char stm_tx_buffer[STM_BUFFER_SIZE] = {0};
         unsigned char stm_rx_buffer[STM_BUFFER_SIZE] = {0};
         buildI2CMessage(stm_tx_buffer, true, 0x04, 0x00, 0x0000);
         writeToSTM32(fd_stm32, stm_tx_buffer, STM_BUFFER_SIZE);
         usleep(MESSAGE_DELAY_US);
         readFromSTM32(fd_stm32, stm_rx_buffer, STM_BUFFER_SIZE);
         uint8_t header = stm_rx_buffer[0];
         uint8_t rw = header >> 7; 
         uint8_t type = header & 0x7F; 
         uint8_t index = stm_rx_buffer[1];
         uint16_t data = (stm_rx_buffer[2] << 8) | stm_rx_buffer[3];
         if (type == 0x05 && rw == 0x00){
             sendUpdatedValueToRedis("simulation_power", 1);
         }
         else{
             sendUpdatedValueToRedis("simulation_power", 0);
         }
         redisCheckCounter++;
         if (redisCheckCounter >= 10) {
             redisCheckCounter = 0;
             std::string redisValue;
             int rc;
             
             rc = getDataFromRedis(redis_host, redis_port, "auger_bottom_pivot_angle", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 uint16_t newVal = std::atoi(redisValue.c_str());
                 if (newVal != augerBottomPivotAngle) {
                     augerBottomPivotAngle = newVal;
                     writeDAC(fd_dac, 0, augerBottomPivotAngle);
                 }
             }
             
             rc = getDataFromRedis(redis_host, redis_port, "auger_top_angle", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 uint16_t newVal = std::atoi(redisValue.c_str());
                 if (newVal != augerTopAngle) {
                     augerTopAngle = newVal;
                     writeDAC(fd_dac, 1, augerTopAngle);
                 }
             }
             
             rc = getDataFromRedis(redis_host, redis_port, "spout_tilt_angle", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 uint16_t newVal = std::atoi(redisValue.c_str());
                 if (newVal != spoutTiltAngle) {
                     spoutTiltAngle = newVal;
                     writeDAC(fd_dac, 2, spoutTiltAngle);
                 }
             }
             
             rc = getDataFromRedis(redis_host, redis_port, "head_rotation_angle", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 uint16_t newVal = std::atoi(redisValue.c_str());
                 if (newVal != headRotationAngle) {
                     headRotationAngle = newVal;
                     writeDAC(fd_dac, 3, headRotationAngle);
                 }
             }
 
             rc = getDataFromRedis(redis_host, redis_port, "gate_angle", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 uint16_t newVal = std::atoi(redisValue.c_str());
                 if (newVal != gateAngle) {
                     gateAngle = newVal;
                     writeDAC(fd_dac, 0x04, gateAngle);
                 }
             }
 
             rc = getDataFromRedis(redis_host, redis_port, "front_weight", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 uint16_t newVal = std::atoi(redisValue.c_str());
                 if (newVal != gateAngle) {
                     frontWeight = newVal;
                     uint16_t leftFrontWeight = frontWeight/2;
                     uint16_t rightFrontWeight = frontWeight/2;
                     unsigned char stm_tx_buffer[STM_BUFFER_SIZE] = {0};
                     buildI2CMessage(stm_tx_buffer, false, 0x03, 0x00, 0x0000);
                     writeToSTM32(fd_stm32, stm_tx_buffer, 1);
                     buildI2CMessage(stm_tx_buffer, false, 0x03, 0x01, 0x0000);
                     writeToSTM32(fd_stm32, stm_tx_buffer, 1);
                 }
             }
             
             rc = getDataFromRedis(redis_host, redis_port, "rear_weight", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 uint16_t newVal = std::atoi(redisValue.c_str());
                 if (newVal != gateAngle) {
                     rearWeight = newVal;
                     uint16_t leftRearWeight = rearWeight/2;
                     uint16_t rightRearWeight = rearWeight/2;
                     unsigned char stm_tx_buffer[STM_BUFFER_SIZE] = {0};
                     buildI2CMessage(stm_tx_buffer, false, 0x03, 0x02, 0x0000);
                     writeToSTM32(fd_stm32, stm_tx_buffer, 1);
                     buildI2CMessage(stm_tx_buffer, false, 0x03, 0x03, 0x0000);
                     writeToSTM32(fd_stm32, stm_tx_buffer, 1);
                 }
             }
 
             rc = getDataFromRedis(redis_host, redis_port, "pto", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 int newPTO = std::atoi(redisValue.c_str());
                 if (newPTO != ptoOnOff) {
                     ptoOnOff = newPTO;
                     unsigned char ptoCommand = static_cast<unsigned char>(ptoOnOff);
                     writeToSTM32(fd_stm32, &ptoCommand, 1);
                 }
             }
             // Note wheel speed not implemented due to no database entries pertaining to that existing
             
             rc = getDataFromRedis(redis_host, redis_port, "crop_fill_rate", redisValue, "");
             if (rc == 0 && !redisValue.empty()) {
                 int newVal = std::atoi(redisValue.c_str());
                 if (newVal != cropFillRate) {
                     cropFillRate = newVal;
                 }
             }
             
         }
     }
     
     cleanup();
     sendUpdatedValueToRedis("simulation_power", 0);
     return 0;
 }
