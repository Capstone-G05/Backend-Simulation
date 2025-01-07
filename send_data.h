#ifndef SEND_DATA_H
#define SEND_DATA_H

#include <string>

// Included outside of main program for testing purposes
int sendDataToRedis(const std::string &hostname,
                    int port,
                    const std::string &key,
                    const std::string &value,
                    const std::string &password);

#endif
