# Backend Simulation - ECE 4600 G05 Graincart Project

## Overview

### Description
This portion of the project is the backend simulation. It serves to decode the data sent from the I2C connection, format that data so it can be sent to the Redis server, and then send it to the Redis server, where it will eventually be presented in the frontend simulation.

### I2C Connection
The current configuration of the I2C connection uses GPIO 2 and GPIO 3 on the Raspberry Pi for SDA and SCL, respectively. To set up these pins for I2C, you must enable I2C under **Interface Options** on the Raspberry Pi. Note that a common ground must be shared between the Raspberry Pi and the peripheral device for this setup to work properly.

### I2C Received Datagram Format
The I2C format for this simulation reads two bytes from the device connected to the I2C pins. The first byte is an integer that corresponds to one of the adjustable variables in the simulator, and the second byte is a flag indicating whether the variable should be incremented or decremented.

### I2C Sent Datagram Format
The data sent via the I2C connection to the STM32 peripheral uses a slightly different format due to the higher precision required for PWM data. First, one byte containing the variable ID is sent, followed by two bytes that represent the 16-bit PWM value. PTO data is sent in a similar manner, but only requires two bytes in total: one byte for the variable ID and one byte for the boolean PTO status.

### Simulation Parameters
The code stores default values for each parameter, which are incremented or decremented based on I2C input. The simulation ensures that all updates reflect the observed behavior from our site visit and do not exceed the defined minimum and maximum values for each parameter.

### Redis Connection
The final portion of the simulation sends the data to the Redis server, where it is accessible by the frontend. To achieve this, we developed functions (using the **hiredis** library) to establish a Redis connection and update the database with the simulation results. We also created functions to receive data from Redis, allowing the simulation to pass updated values back to the STM32.

## Files

- **redis_interface.cpp**: Contains the functions that send and receive data to and from Redis.
- **redis_interface.h**: The header file that exposes the functions defined in `redis_interface.cpp`.
- **i2c_interface.cpp**: Contains the functions that send and receive data to and from the I2C connection.
- **i2c_interface.h**: The header file that exposes the functions defined in `i2c_interface.cpp`.
- **simulation.cpp**: Contains most of the simulation code and compiles into the executable that runs continuously on the Raspberry Pi.
- **Makefile**: Used to compile all of the above files. Running this Makefile generates the `simulation` executable, which should be run to start the simulation.
- **config.txt**: Holds constant values that initialize simulation variables at startup.

## How to Start

1. Ensure that the `hiredis` library is installed by running the command: `sudo apt-get install libhiredis-dev` and ensure that the Redis server is running at `127.0.0.1:6379`. Also ensure that the `i2c-dev` libraries are installed by running the command `sudo apt-get install libi2c-dev` and ensure that the peripheral device is running at address 2 while connected to the I2C pins on the raspberry pi.
2. Run the command: `make` in the directory containing the project files.
3. Run the command: `./simulation` within the directory that it is contained in to activate the simulation.
4. When the executable is no longer needed, run the command: `make clean` to remove the compiled files.