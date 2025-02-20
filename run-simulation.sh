#!/bin/bash

# recompile the simulation
make clean
make

# start the simulation
./simulation 2>&1