# Backend Simulation - ECE 4600 G05 Graincart Project

## Overview

### Description
This portion of the project is the backend simulation, and it serves to properly decode the data sent from the I2C connection, format that data so it can be sent to the Redis server, and finally send the data to the Redis server, where it will eventually be presented on the frontend simulation.

### I2C Connection
The current configuration of the I2C connection uses the designated I2C pins on the Raspberry Pi. These pins are GPIO 2 and GPIO 3 for the SDA and SCL pins, respectively. To set up these pins for I2C, you must enable I2C on the Raspberry Pi. To do this, go into the **Interface Options** on the Raspberry Pi and enable I2C. Note that a common ground must be connected to both the Raspberry Pi and the device for this setup to work.

### I2C Datagram Format
The I2C format designed for this simulation takes in two bytes from the device connected to the I2C pins. The first byte is an integer that maps to one of the changeable variables in our simulator, and the second byte is a flag indicating whether the variable should be incremented or decremented.

### Simulation Parameters
The code contains saved values for each parameter. These parameters are updated by incrementing or decrementing them based on the values received from the I2C connection. The simulation ensures that all updates comply with the observed behavior during the site visit and that they do not exceed the defined minimum and maximum values for each parameter.

### Redis Connection
The final portion of the simulation sends the data to the Redis server, where it will be easily accessible by the frontend simulation. To achieve this, we developed a function that establishes a Redis connection and uses the API pathways created in the Redis server to set the database values based on the simulation results. These Redis functions are provided by the "hiredis" library.

## Files

- **send_data.cpp**: Contains the function that sends data to Redis.
- **send_data.h**: The header file that makes the function in `send_data.cpp` accessible.
- **simulation.cpp**: The file containing most of the simulation code. This file is compiled into the executable that runs constantly on the Raspberry Pi.
- **Makefile**: The makefile used to compile all of the above files. Executing this makefile generates the `simulation` executable, which should be run to activate the simulation.

## How to Start

1. Ensure that the `hiredis` library is installed by running the command: `sudo apt-get install libhiredis-dev`. Also, ensure that the Redis server is running at `127.0.0.1:6379`.
2. Run the command: `make` in the directory containing the project files.
3. Run the command: `./simulation` within the directory that it is contained in to activate the simulation.
4. When the executable is no longer needed, run the command: `make clean` to remove the compiled files.
