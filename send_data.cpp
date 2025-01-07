// send_data.cpp

#include "send_data.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <hiredis/hiredis.h>

int sendDataToRedis(const std::string &hostname,
                    int port,
                    const std::string &key,
                    const std::string &value,
                    const std::string &password)
{
    redisContext *c = redisConnect(hostname.c_str(), port);
    if (c == nullptr || c->err) {
        if (c) {
            std::cerr << "Connection error: " << c->errstr << std::endl;
            redisFree(c);
        } else {
            std::cerr << "Connection error: can't allocate redis context." << std::endl;
        }
        return -1;
    }

    if (!password.empty()) {
        redisReply *authReply = (redisReply *)redisCommand(c, "AUTH %s", password.c_str());
        if (!authReply || authReply->type == REDIS_REPLY_ERROR) {
            std::cerr << "Redis AUTH failed. Check password or Redis config.";
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

    redisReply *reply = (redisReply *)redisCommand(c, "SET %s %s", key.c_str(), value.c_str());
    if (!reply) {
        std::cerr << "Command error: " << c->errstr << std::endl;
        redisFree(c);
        return -1;
    }

    if (reply->type == REDIS_REPLY_STATUS) {
        std::cout << "SET command result: " << reply->str << std::endl;
    }

    freeReplyObject(reply);
    redisFree(c);

    return 0;
}
