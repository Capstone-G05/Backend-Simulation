#include <iostream>
#include <string>
#include <time.h>
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
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <hiredis/hiredis.h>

/* DEFINES */

//#define LOCAL  // for testing with no I2C bus
#define DEBUG  // for printing debug statements

#define I2C_DEVICE "/dev/i2c-1"
#define STM32_I2C_ADDRESS 0x10
#define DAC_I2C_ADDRESS   0x60

#define DAC_MAX_REGISTER   0x1F
#define DAC_WRITE_CMD_MASK 0x00
#define DAC_READ_CMD_MASK  0x06
#define DAC_VREF_MODE      0x10

#define STM32_TX_BUFFER_SIZE 4
#define STM32_RX_BUFFER_SIZE 4
#define DAC_TX_BUFFER_SIZE   3
#define DAC_RX_BUFFER_SIZE   2

#define MAX_STRING_LENGTH 50

#define I2C_MESSAGE_DELAY_US 10000

#define FRAME_RATE_DEFAULT 60

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
  {"PIVOT_UP_PWM",   0x02, 0x08},
  {"PIVOT_DOWN_PWM", 0x02, 0x04},
  {"FOLD_IN_PWM",    0x02, 0x0A},
  {"FOLD_OUT_PWM",   0x02, 0x0B},
  {"TILT_UP_PWM",    0x02, 0x01},
  {"TILT_DOWN_PWM",  0x02, 0x03},
  {"ROTATE_CW_PWM",  0x02, 0x06},
  {"ROTATE_CCW_PWM", 0x02, 0x07},
  {"GATE_OPEN_PWM",  0x02, 0x05},
  {"GATE_CLOSE_PWM", 0x02, 0x09},
  {"ONLINE",         0x04, 0x00},
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
clock_t start, end;

const size_t STM32_PARAMETERS_SIZE = sizeof(stm32_parameters) / sizeof(STM32Parameter);
const size_t DAC_PARAMETERS_SIZE = sizeof(dac_parameters) / sizeof(DACParameter);

int i2c_fd = -1;
redisContext *redis_context = nullptr;

const char *redis_host_env = getenv("REDIS_HOST");
const char *redis_port_env = getenv("REDIS_PORT");

const char *redis_host = redis_host_env ? redis_host_env : "localhost";
const uint16_t redis_port = redis_port_env ? atoi(redis_port_env) : 6379;

const uint16_t ANGLE_CENTER = 180;

int16_t angle[NUM_AUGER_MOVEMENTS];
int16_t angle_min[NUM_AUGER_MOVEMENTS];
int16_t angle_max[NUM_AUGER_MOVEMENTS];
int16_t angle_range[NUM_AUGER_MOVEMENTS];
int16_t angle_start[NUM_AUGER_MOVEMENTS];
int16_t speed_ref[NUM_AUGER_MOVEMENTS];
uint16_t weight[NUM_WEIGHT_LOCATIONS];

uint16_t pto_speed = 0;
uint16_t crop_fill_rate = 0;
uint16_t pto_flow_rate = 0;

struct timeval time_stamp;
uint32_t time_elapsed_ms;

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

void RedisRequest(int16_t*, uint8_t, const char*);
void STM32Update(const char*, uint8_t, uint8_t);
void DACConfig(void);
void DACUpdate(const char*, uint8_t, uint8_t);

void WeightUpdate(void);
void AngleUpdate(void);

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
  // Attempt to set ONLINE status to 0 (ie. "offline")
  if (RedisSet(redis_context, "ONLINE", "0") < 0) {
    printf("Failed to set status 0 (ie. 'OFFLINE')");
  }
  // Close I2C connections
  if (i2c_fd >= 0) {
    close(i2c_fd);
  }
  // Close Redis connection
  if (redis_context != nullptr) {
    redisFree(redis_context);
  }
  // Exit with ERROR_CODE
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
#ifdef LOCAL
  return 0;
#endif
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
#ifdef LOCAL
  return 0;
#endif
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    perror("Failed to set I2C address");
    return -1;
  }
  return 0;
}

/*

*/
int I2CWrite(int fd, uint8_t *buffer, ssize_t length) {
#ifdef LOCAL
  return 0;
#endif
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
#ifdef LOCAL
  return 0;
#endif
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
int RedisConnect(redisContext **context, const char *hostname, uint16_t port) {
  *context = redisConnect(hostname, port);
  if (*context == nullptr || (*context)->err) {
    if (*context) {
      printf("ERROR: %s\n", (*context)->errstr);
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
  if (context == nullptr) {
    perror("NULL context pointer");
    return -1;
  }
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
  if (context == nullptr) {
    perror("NULL context pointer");
    return -1;
  }
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
void RedisRequest(int16_t *data, uint8_t movement, const char *type) {
  char redis_key[MAX_STRING_LENGTH];
  char redis_value[MAX_STRING_LENGTH];

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
void STM32Update(const char *key, uint8_t type, uint8_t index) {
  uint8_t tx_buffer[STM32_TX_BUFFER_SIZE];
  uint8_t rx_buffer[STM32_RX_BUFFER_SIZE];
  char redis_value[MAX_STRING_LENGTH];

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
void DACConfig(void){
    uint8_t tx_buffer[DAC_TX_BUFFER_SIZE];

    if (SetI2CAddress(i2c_fd, DAC_I2C_ADDRESS) < 0){
        CleanupAndExit(EXIT_FAILURE);
    }

    DACPack(tx_buffer, 0x08, DAC_VREF_MODE, DAC_WRITE_CMD_MASK);
    I2CWrite(i2c_fd, tx_buffer, DAC_TX_BUFFER_SIZE);
}

/*

*/
void DACUpdate(const char *key, uint8_t movement, uint8_t index) {
    uint8_t tx_buffer[DAC_TX_BUFFER_SIZE];
    char redis_value[MAX_STRING_LENGTH];

    if (SetI2CAddress(i2c_fd, DAC_I2C_ADDRESS) < 0){
        CleanupAndExit(EXIT_FAILURE);
    }

    if (RedisGet(redis_context, key, redis_value) < 0) {
        CleanupAndExit(EXIT_FAILURE);
    }

    angle[movement] = atoi(redis_value);
    int16_t absolute_angle = angle[movement] - angle_min[movement];
    int16_t adjusted_angle = angle_start[movement] + absolute_angle;
    uint16_t angle_voltage = (uint16_t)((float(adjusted_angle) / 360.0) * 1023);

#ifdef DEBUG
    printf("%d: ANGLE=%d MIN=%d MAX=%d RANGE=%d START=%d ABS=%d ADJ=%d VOLT=%d\n", index, angle[movement], angle_min[movement], angle_max[movement], angle_range[movement], angle_start[movement], absolute_angle, adjusted_angle, angle_voltage);
#endif

    DACPack(tx_buffer, index, angle_voltage, DAC_WRITE_CMD_MASK);
    I2CWrite(i2c_fd, tx_buffer, DAC_TX_BUFFER_SIZE);
}

/*

*/
void WeightUpdate(void) {
  uint8_t tx_buffer[STM32_TX_BUFFER_SIZE];
  uint8_t rx_buffer[STM32_RX_BUFFER_SIZE];
  char redis_string[MAX_STRING_LENGTH];

  uint16_t weight_removed = 0;
  uint16_t weight_front = weight[FRONT];
  uint16_t weight_rear = weight[REAR];
  if (pto_speed > 0 && pto_flow_rate > 0) {
    weight_removed = (uint16_t)(((float)(pto_speed * pto_flow_rate) / 3600.0 / 1000.0) * time_elapsed_ms);
    if (weight_removed > 0) {
      if (weight_front < (weight_removed / 4)) {
        weight_front = 0;
      } else {
        weight_front -= weight_removed / 4;
      }
      if (weight_rear < (weight_removed / 4)) {
        weight_rear = 0;
      } else {
        weight_rear -= weight_removed / 4;
      }
    }
  }

#ifdef DEBUG
  printf("PTO: SPEED=%d RATE=%d\n", pto_speed, pto_flow_rate);
  printf("WEIGHT: REMOVED=%d FRONT=%d REAR=%d\n", weight_removed, weight_front, weight_rear);
#endif

  sprintf(redis_string, "%u", weight_front);
  RedisSet(redis_context, "WEIGHT_FRONT", redis_string);
  sprintf(redis_string, "%u", weight_rear);
  RedisSet(redis_context, "WEIGHT_REAR", redis_string);

  if (SetI2CAddress(i2c_fd, STM32_I2C_ADDRESS) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }

  STM32Pack(tx_buffer, WRITE, 0x03, 0x00, weight_front);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  STM32Pack(tx_buffer, WRITE, 0x03, 0x01, weight_front);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  STM32Pack(tx_buffer, WRITE, 0x03, 0x02, weight_rear);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  STM32Pack(tx_buffer, WRITE, 0x03, 0x03, weight_rear);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);

  // TODO: handle hitch weight

  STM32Pack(tx_buffer, WRITE, 0x03, 0x04, 0);
  I2CWrite(i2c_fd, tx_buffer, STM32_TX_BUFFER_SIZE);
  usleep(I2C_MESSAGE_DELAY_US);
  I2CRead(i2c_fd, rx_buffer, STM32_RX_BUFFER_SIZE);
}

/*

*/
void AngleUpdate(void) {
  // TODO: get FRAME_RATE from Redis (or use FRAME_RATE_DEFAULT if null)
  // TODO: save PWM locally in STM32Request()
  // TODO: extrapolate new angle values based on FRAME_RATE, SPEED_REF, and angle[i]
}

int main(void) {
  setbuf(stdout, NULL); // disable stdout buffering
  printf("~ Starting Simulation ~\n");
  signal(SIGINT, HandleSigint);

  // Initialize I2C device
  if ((i2c_fd = OpenI2CDevice()) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }
  printf("Initialized I2C device\n");

  // Initialize Redis connection
  if (RedisConnect(&redis_context, redis_host, redis_port) < 0) {
    CleanupAndExit(EXIT_FAILURE);
  }
  printf("Initialized Redis connection\n");

  // Configure DAC
  // DACConfig(); // TODO: there may be a bug here (pivot -> 5V), not sure...

  gettimeofday(&time_stamp,NULL);
  char redis_value[MAX_STRING_LENGTH];

  printf("Starting main loop\n");
  while (true) {
    // Update time stamp and calculate system loop duration
    struct timeval next_time_stamp;
    gettimeofday(&next_time_stamp,NULL);
    time_elapsed_ms = ((next_time_stamp.tv_sec - time_stamp.tv_sec) * 1000) + ((next_time_stamp.tv_usec - time_stamp.tv_usec) / 1000);
    time_stamp = next_time_stamp;

#ifdef DEBUG
    printf("Time Elapsed: %dms\n", time_elapsed_ms);
#endif

    // Get simulation 'constants 'from Redis
    for (size_t i = 0; i < NUM_AUGER_MOVEMENTS; i++) {
      RedisRequest(angle_min, i, "_ANGLE_MIN");
      RedisRequest(angle_max, i, "_ANGLE_MAX");
      RedisRequest(speed_ref, i, "_SPEED_REFERENCE");
      angle_range[i] = angle_max[i] - angle_min[i];
      angle_start[i] = ANGLE_CENTER - (angle_range[i] / 2);
    }

    // Interpolate simulation angles in tandem with frontend simulation
    AngleUpdate();

    // Get simulation 'parameters' from Redis
    if (RedisGet(redis_context, "PTO_SPEED", redis_value) < 0){
      CleanupAndExit(EXIT_FAILURE);
    }
    pto_speed = atoi(redis_value);
    if (RedisGet(redis_context, "PTO_FLOW_RATE", redis_value) < 0){
      CleanupAndExit(EXIT_FAILURE);
    }
    pto_flow_rate = atoi(redis_value);
    if (RedisGet(redis_context, "WEIGHT_FRONT", redis_value) < 0) {
      CleanupAndExit(EXIT_FAILURE);
    }
    weight[FRONT] = atoi(redis_value);
    if (RedisGet(redis_context, "WEIGHT_REAR", redis_value) < 0) {
      CleanupAndExit(EXIT_FAILURE);
    }
    weight[REAR] = atoi(redis_value);

    // Calculate simulation weights and send to STM32 & Redis
    WeightUpdate();

    // Send simulation 'outputs' to STM32
    STM32Update("PTO_SPEED", 0x01, 0x01);
    // TODO: wheel speed & direction

    // Get PWM values and processor status from the STM32 and send to Redis
    for (size_t i = 0; i < STM32_PARAMETERS_SIZE; i++) {
      STM32Request(stm32_parameters[i].key, stm32_parameters[i].type, stm32_parameters[i].index);
    }

    // Send angle values to the DAC
    for (size_t i = 0; i < DAC_PARAMETERS_SIZE; i++) {
      DACUpdate(dac_parameters[i].key, i, dac_parameters[i].index);
    }
  }

  CleanupAndExit(EXIT_SUCCESS); // unreachable
}
