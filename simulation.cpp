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

/* DEFINES */

#define DEBUG  // for printing debug statements

#define I2C_DEVICE "/dev/i2c-1"
#define STM32_I2C_ADDRESS 0x10
#define DAC_I2C_ADDRESS   0x60

#define DAC_MAX_REGISTER   0x1F
#define DAC_WRITE_CMD_MASK 0x00
#define DAC_READ_CMD_MASK  0x06

#define STM32_TX_BUFFER_SIZE 4
#define STM32_RX_BUFFER_SIZE 4
#define DAC_TX_BUFFER_SIZE   3
#define DAC_RX_BUFFER_SIZE   2

#define MAX_STRING_LENGTH 50

#define I2C_MESSAGE_DELAY_US 10000
#define POLLING_INTERVAL_US  100000

/* TYPEDEFS */

typedef enum {
  READ = 1,
  WRITE = 0,
} ReadWrite;

typedef enum {
  PIVOT,
  FOLD,
  TILT,
  ROTATE,
  GATE,
  NUM_AUGER_MOVEMENTS
} AugerMovement;

typedef enum {
  FRONT,
  REAR,
  NUM_WEIGHT_LOCATIONS
} WeightLocation;

typedef struct {
  uint8_t rw : 1;
  uint8_t type : 7;
  uint8_t index;
  uint16_t value;
} STM32Message;

typedef struct {
  char key[MAX_STRING_LENGTH];
  uint8_t type;
  uint8_t index;
} STM32Parameter;

typedef struct {
  char key[MAX_STRING_LENGTH];
  AugerMovement movement;
  uint8_t index;
} DACParameter;

const STM32Parameter stm32_parameters[] = {
  {"PIVOT_UP",     0x02, 0x08},
  {"PIVOT_DOWN",   0x02, 0x04},
  {"AUGER_FOLD",   0x02, 0x0A},
  {"AUGER_UNFOLD", 0x02, 0x0B},
  {"TILT_UP",      0x02, 0x01},
  {"TILT_DOWN",    0x02, 0x03},
  {"ROTATE_CW",    0x02, 0x06},
  {"ROTATE_CCW",   0x02, 0x07},
  {"GATE_OPEN",    0x02, 0x05},
  {"GATE_CLOSE",   0x02, 0x09},
  {"ONLINE",       0x04, 0x00},
};

const DACParameter dac_parameters[] = {
  {"PIVOT_ANGLE",  PIVOT,  0x02},
  {"FOLD_ANGLE",   FOLD,   0x05},
  {"ROTATE_ANGLE", ROTATE, 0x00},
  {"GATE_ANGLE",   GATE,   0x04},
};

const char *AugerMovementNames[] = {
  "PIVOT",
  "FOLD",
  "TILT",
  "ROTATE",
  "GATE",
};

/* PRIVATE VARIABLES */

const size_t STM32_PARAMETERS_SIZE = sizeof(stm32_parameters) / sizeof(STM32Parameter);
const size_t DAC_PARAMETERS_SIZE = sizeof(dac_parameters) / sizeof(DACParameter);

int i2c_fd = -1;
redisContext *redis_context = nullptr;

const char *redis_host_env = getenv("REDIS_HOST");
const char *redis_port_env = getenv("REDIS_PORT");

const char *redis_host = redis_host_env ? redis_host_env : "localhost";
const uint16_t redis_port = redis_port_env ? atoi(redis_port_env) : 6379;

uint16_t angle[NUM_AUGER_MOVEMENTS];
uint16_t angle_min[NUM_AUGER_MOVEMENTS];
uint16_t angle_max[NUM_AUGER_MOVEMENTS];
uint16_t speed_ref[NUM_AUGER_MOVEMENTS];
uint16_t weight[NUM_WEIGHT_LOCATIONS];

uint16_t pto_speed = 0;
uint16_t crop_fill_rate = 0;

/* FUNCTION PROTOTYPES */

void InitializeData(void);

void CleanupAndExit(void);
void HandleSigint(int);

int OpenI2CDevice(void);
int SetI2CAddress(int, uint8_t);
int I2CWrite(int, uint8_t*, ssize_t);
int I2CRead(int, uint8_t*, ssize_t);

void STM32Pack(uint8_t*, uint8_t, uint8_t, uint8_t, uint16_t);
void STM32Parse(uint8_t*, STM32Message*);
void DACPack(uint8_t*, uint8_t, uint16_t, uint8_t);
void DACParse(uint8_t*, uint16_t*);

int RedisConnect(redisContext*, char*, uint16_t);
int RedisSet(redisContext*, const char*, const char*);
int RedisGet(redisContext*, const char*, char*);

void RedisRequest(uint16_t*, AugerMovement, const char*);
void STM32Update(const char*, uint8_t, uint8_t);
void DACUpdate(const char*, uint8_t);
void WeightUpdate(void);

/* FUNCTION IMPLEMENTATIONS */

/*

*/
void InitializeData(void) {
  uint16_t default_value = 0;

  for (size_t i = 0; i < NUM_AUGER_MOVEMENTS; i++) {
    angle[i] = default_value;
    angle_min[i] = default_value;
    angle_max[i] = default_value;
    speed_ref[i] = default_value;
  }

  for (size_t i = 0; i < NUM_WEIGHT_LOCATIONS; i++) {
    weight[i] = default_value;
  }
}

/*

*/
void CleanupAndExit(int exit_code) {
  if (i2c_fd >= 0) {
    close(i2c_fd);
  }
  if (redis_context != nullptr) {
    redisFree(redis_context);
  }
  exit(exit_code);
}

/*

*/
void HandleSigint(int sig) {
  printf("\nCaught signal %d. Cleaning up and exiting.\n", sig);
  CleanupAndExit(EXIT_SUCCESS);
}

/*

*/
int OpenI2CDevice(void) {
  int fd = open(I2C_DEVICE, O_RDWR);
  if (fd < 0) {
    perror("Failed to open I2C bus");
    return -1;
  }
  return fd;
}

/*

*/
int SetI2CAddress(int fd, uint8_t address) {
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    perror("Failed to set I2C address");
    return -1;
  }
  return 0;
}

/*

*/
int I2CWrite(int fd, uint8_t *buffer, ssize_t length) {
  if (write(fd, buffer, length) != length) {
    perror("Failed I2C write");
    return -1;
  }
#ifdef DEBUG
  printf("TX:");
  for (ssize_t i = 0; i < length; i++) {
    printf(" %02X", buffer[i]);
  }
  printf("\n");
#endif
  return 0;
}

/*

*/
int I2CRead(int fd, uint8_t *buffer, ssize_t length) {
  if (read(fd, buffer, length) != length) {
    perror("Failed I2C read");
    return -1;
  }
#ifdef DEBUG
  printf("RX:");
  for (ssize_t i = 0; i < length; i++) {
    printf(" %02X", buffer[i]);
  }
  printf("\n");
#endif
  return 0;
}

/*

*/
void STM32Pack(uint8_t *buffer, uint8_t rw, uint8_t type, uint8_t index, uint16_t value) {
  buffer[0] = ((rw ? 1 : 0) << 7) | (type & 0x7F);
  buffer[1] = index;
  buffer[2] = (value >> 8) & 0xFF;
  buffer[3] = value & 0xFF;
}

/*

*/
void STM32Parse(uint8_t *buffer, STM32Message *message) {
  message->rw = buffer[0] >> 7;
  message->type = buffer[0] & 0x7F;
  message->index = buffer[1];
  message->value = (buffer[2] << 8) | buffer[3];
}

/*

*/
void DACPack(uint8_t *buffer, uint8_t reg, uint16_t value, uint8_t cmd) {
  buffer[0] = ((reg & DAC_MAX_REGISTER) << 3) | cmd;
  buffer[1] = (value >> 8) & 0xFF;
  buffer[2] = value & 0xFF;
}

/*

*/
void DACParse(uint8_t *buffer, uint16_t *message) {
  *message = (buffer[0] << 8) | buffer[1];
}

/*

*/
int RedisConnect(redisContext *context, const char *hostname, uint16_t port) {
  printf("Connecting to Redis...");

  context = redisConnect(hostname, port);
  if (context == nullptr || context->err) {
    if (context) {
      printf("ERROR: %s\n", context->errstr);
      CleanupAndExit(EXIT_FAILURE);
    } else {
      printf("Can't allocate redis context\n");
      CleanupAndExit(EXIT_FAILURE);
    }
    return -1;
  }
  return 0;
}

/*

*/
int RedisSet(redisContext *context, const char *key, const char *value) {
  redisReply *reply = (redisReply *)redisCommand(context, "SET %s %s", key, value);
  if (!reply) {
    printf("ERROR: %s\n", context->errstr);
    return -1;
  }
#ifdef DEBUG
  if (reply->type == REDIS_REPLY_STATUS) {
    printf("SET: %s => %s (%s)\n", key, value, reply->str);
  }
#endif
  freeReplyObject(reply);
  return 0;
}

/*

*/
int RedisGet(redisContext *context, const char *key, char *value) {
  redisReply *reply = (redisReply *)redisCommand(context, "GET %s", key);
  if (!reply) {
    printf("ERROR: %s\n", context->errstr);
    return -1;
  }
  if (reply->type == REDIS_REPLY_STRING) {
    strcpy(value, reply->str);
  } else {
    value = {0};
  }
#ifdef DEBUG
  if (reply->type == REDIS_REPLY_STATUS) {
    printf("GET: %s => %s (%s)\n", key, value, reply->str);
  }
#endif
  freeReplyObject(reply);
  return 0;
}

/*

*/
void RedisRequest(uint16_t *data, AugerMovement movement, const char *type) {
  char redis_key[MAX_STRING_LENGTH];
  char redis_value[MAX_STRING_LENGTH] = {0};

  snprintf(redis_key, sizeof(redis_key), "%s%s", AugerMovementNames[movement], type);
  if (RedisGet(redis_context, redis_key, redis_value) < 0) {
    perror("Failed to read Redis");
  }
  data[movement] = atoi(redis_value);
}

/*

*/
void STM32Request(const char *key, uint8_t type, uint8_t index) {
  uint8_t tx_buffer[STM32_TX_BUFFER_SIZE];
  uint8_t rx_buffer[STM32_RX_BUFFER_SIZE];
  STM32Message rx_message;

  if (SetI2CAddress(i2c_fd, STM32_I2C_ADDRESS) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  STM32Pack(tx_buffer, READ, type, index, 0);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);
  STM32Parse(rx_buffer, &rx_message);

  char rx_string[MAX_STRING_LENGTH];
  sprintf(rx_string, "%u", rx_message.value);
  RedisSet(redis_context, key, rx_string);
}

/*

*/
void STM32Update(char *key, uint8_t type, uint8_t index) {
  uint8_t tx_buffer[STM32_TX_BUFFER_SIZE];
  uint8_t rx_buffer[STM32_RX_BUFFER_SIZE];
  char redis_value[MAX_STRING_LENGTH] = {0};

  if (RedisGet(redis_context, key, redis_value) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  if (SetI2CAddress(i2c_fd, STM32_I2C_ADDRESS) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  STM32Pack(tx_buffer, WRITE, type, index, atoi(redis_value));
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);
}

/*

*/
void WeightUpdate(void) {
  uint8_t tx_buffer[STM32_TX_BUFFER_SIZE];
  uint8_t rx_buffer[STM32_RX_BUFFER_SIZE];
  char redis_value[MAX_STRING_LENGTH] = {0};

  if (SetI2CAddress(i2c_fd, STM32_I2C_ADDRESS) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  if (RedisGet(redis_context, "WEIGHT_FRONT", redis_value) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  STM32Pack(tx_buffer, WRITE, 0x03, 0x00, atoi(redis_value) / 2);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  usleep(I2C_MESSAGE_DELAY_US);

  STM32Pack(tx_buffer, WRITE, 0x03, 0x01, atoi(redis_value) / 2);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  usleep(I2C_MESSAGE_DELAY_US);

  if (RedisGet(redis_context, "WEIGHT_REAR", redis_value) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  STM32Pack(tx_buffer, WRITE, 0x03, 0x02, atoi(redis_value) / 2);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  usleep(I2C_MESSAGE_DELAY_US);

  STM32Pack(tx_buffer, WRITE, 0x03, 0x03, atoi(redis_value) / 2);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  usleep(I2C_MESSAGE_DELAY_US);

  // TODO: hitch weight (???)

  STM32Pack(tx_buffer, WRITE, 0x03, 0x04, 0);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  usleep(I2C_MESSAGE_DELAY_US);
}

/*

*/
void DACUpdate(const char *key, uint8_t index) {
  uint8_t tx_buffer[DAC_TX_BUFFER_SIZE];
  char redis_value[MAX_STRING_LENGTH] = {0};

  if (RedisGet(redis_context, key, redis_value) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  if (SetI2CAddress(i2c_fd, DAC_I2C_ADDRESS) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  DACPack(tx_buffer, index, atoi(redis_value), DAC_WRITE_CMD_MASK);
  I2CWrite(i2c_fd, tx_buffer, DAC_TX_BUFFER_SIZE);
}


int main() {
  printf("~ Starting Simulation ~\n");
  signal(SIGINT, HandleSigint);

  if ((i2c_fd = OpenI2CDevice()) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }
  printf("Initialized I2C device");

//  if (RedisConnect(redis_context, redis_host, redis_port) < 0) {
//    CleanupAndExit(EXIT_FAILURE);
//  }
  printf("Initialized Redis connection");

  for (size_t i = 0; i < NUM_AUGER_MOVEMENTS; i++) {
    RedisRequest(angle_min, static_cast<AugerMovement>(i), "_ANGLE_MIN");
    RedisRequest(angle_max, static_cast<AugerMovement>(i), "_ANGLE_MAX");
    RedisRequest(speed_ref, static_cast<AugerMovement>(i), "_SPEED_REF");
  }

  printf("test1\n");

  char redis_value[MAX_STRING_LENGTH] = {0};

  if (RedisGet(redis_context, "WEIGHT_FRONT", redis_value) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }
  weight[FRONT] = atoi(redis_value);

  if (RedisGet(redis_context, "WEIGHT_REAR", redis_value) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }
  weight[REAR] = atoi(redis_value);

  while (true) {

    for (size_t i = 0; i < STM32_PARAMETERS_SIZE; i++) {
      STM32Request(stm32_parameters[i].key, stm32_parameters[i].type, stm32_parameters[i].index);
      usleep(POLLING_INTERVAL_US);
    }

    for (size_t i = 0; i < NUM_AUGER_MOVEMENTS; i++) {
      DACUpdate(dac_parameters[i].key, dac_parameters[i].index);
      usleep(POLLING_INTERVAL_US);
    }

    WeightUpdate();

    STM32Request("PTO_SPEED", 0x01, 0x01);

      // TODO: crop fill rate
  }
  RedisSet(redis_context, "ONLINE", "0");
  CleanupAndExit(EXIT_SUCCESS);
}
