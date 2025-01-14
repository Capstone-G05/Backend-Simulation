/****************************************************************************** 
 * redis_interface.h
 *
 * The header file that splits the redis functions from the main program.
 *
 ******************************************************************************/ 
#ifndef REDIS_INTERFACE_H
#define REDIS_INTERFACE_H

#include <string>

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

#endif
